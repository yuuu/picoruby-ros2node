#include <string.h>

#include <mruby.h>
#include <mruby/data.h>
#include <mruby/string.h>
#include <mruby/array.h>
#include <mruby/variable.h>
#include <mruby/presym.h>
#include <mruby/class.h>
#include <mruby/error.h>

#include <uxr/client/client.h>
#include <uxr/client/util/time.h>
#include <ucdr/microcdr.h>

typedef enum {
  ROS2NODE_IO_UART, /* byte stream, e.g. picoruby-uart UART */
  ROS2NODE_IO_UDP   /* datagram, e.g. picoruby-socket UDPSocket */
} ros2node_io_kind;

/* Reliable stream buffers: MTU (custom-transport, see uxr_config.h) x
 * history slots, both directions. A reliable *input* stream is required
 * even though we never read application data off it ourselves: replies to
 * reliable-output traffic (STATUS for create-entities, ACKNACK bookkeeping)
 * are processed via the reliable-input path, and a best-effort-only input
 * left that path's internal buffer sizing at zero, which crashed with a
 * divide-by-zero (uxr_get_reliable_buffer_size) the first time the agent's
 * reply needed it -- see the tracking issue for the panic report. */
#define ROS2NODE_STREAM_HISTORY 4
#define ROS2NODE_STREAM_BUFFER_SIZE (UXR_CONFIG_CUSTOM_TRANSPORT_MTU * ROS2NODE_STREAM_HISTORY)

typedef struct {
  mrb_state *mrb;
  mrb_value self; /* the ROS2::Node wrapping this struct; used by ros2node_on_topic */
  mrb_value io;
  ros2node_io_kind io_kind;
  uxrCustomTransport transport;
  uxrSession session;
  uxrStreamId reliable_out;
  uxrStreamId reliable_in;
  uxrObjectId participant_id;
  uxrObjectId publisher_id;
  uxrObjectId subscriber_id; /* only valid once has_subscriber is true */
  bool has_subscriber;
  uint16_t next_object_id; /* next free id in the topic/datawriter/datareader namespaces; 1 is the participant/publisher/subscriber */
  uint8_t output_buffer[ROS2NODE_STREAM_BUFFER_SIZE];
  uint8_t input_buffer[ROS2NODE_STREAM_BUFFER_SIZE];
} ros2node_data_t;

static void
ros2node_data_free(mrb_state *mrb, void *ptr)
{
  ros2node_data_t *data = (ros2node_data_t *)ptr;
  if (data) {
    uxr_close_custom_transport(&data->transport);
    mrb_free(mrb, data);
  }
}

static const struct mrb_data_type ros2node_data_type = { "ROS2::Node", ros2node_data_free };

/*
 * io is opened/connected by the caller before it reaches us, so transport
 * open/close are no-ops; they only exist because Micro-XRCE-DDS-Client's
 * callback contract requires them.
 */
static bool
ros2node_transport_open(uxrCustomTransport *transport)
{
  (void)transport;
  return true;
}

static bool
ros2node_transport_close(uxrCustomTransport *transport)
{
  (void)transport;
  return true;
}

static size_t
ros2node_transport_write(uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *error_code)
{
  ros2node_data_t *data = (ros2node_data_t *)transport->args;
  mrb_state *mrb = data->mrb;
  mrb_value str = mrb_str_new(mrb, (const char *)buf, len);
  const char *method = data->io_kind == ROS2NODE_IO_UART ? "write" : "send";
  mrb_value written = mrb_funcall(mrb, data->io, method, 1, str);
  *error_code = 0;
  return mrb_integer_p(written) ? (size_t)mrb_integer(written) : 0;
}

/* Both UART#readpartial and UDPSocket#recvfrom_nonblock are non-blocking
 * (nil if nothing is available yet), so the timeout is enforced here by
 * polling against uxr_millis() rather than by a blocking driver call. */
static size_t
ros2node_transport_read(uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout_ms, uint8_t *error_code)
{
  ros2node_data_t *data = (ros2node_data_t *)transport->args;
  mrb_state *mrb = data->mrb;
  int64_t deadline = uxr_millis() + timeout_ms;

  for (;;) {
    mrb_value chunk;
    if (data->io_kind == ROS2NODE_IO_UART) {
      chunk = mrb_funcall(mrb, data->io, "readpartial", 1, mrb_fixnum_value((mrb_int)len));
    } else {
      /* [data, addr_info] on success, nil otherwise; we only need data. */
      mrb_value result = mrb_funcall(mrb, data->io, "recvfrom_nonblock", 2, mrb_fixnum_value((mrb_int)len), mrb_fixnum_value(0));
      chunk = mrb_array_p(result) ? mrb_ary_ref(mrb, result, 0) : result;
    }
    if (mrb_string_p(chunk) && RSTRING_LEN(chunk) > 0) {
      size_t chunk_len = (size_t)RSTRING_LEN(chunk);
      if (chunk_len > len) chunk_len = len;
      memcpy(buf, RSTRING_PTR(chunk), chunk_len);
      *error_code = 0;
      return chunk_len;
    }
    if (uxr_millis() >= deadline) {
      *error_code = 0;
      return 0;
    }
    mrb_funcall(mrb, data->io, "sleep_ms", 1, mrb_fixnum_value(1));
  }
}

/* Fires (from deep inside Micro-XRCE-DDS-Client's own call stack, via
 * #spin_once -> uxr_run_session_timeout -> listen_message -> ...) whenever
 * a DATA submessage arrives for one of our DataReaders. Deliberately does
 * nothing but a C-level array push here -- no mrb_funcall, no Ruby method
 * dispatch that could raise and longjmp back out through this C library's
 * frames, which do not expect to unwind mid-callback. The actual
 * #subscribe block is invoked later, by #spin_once itself, once
 * uxr_run_session_timeout has returned to safe (non-nested) ground. */
static void
ros2node_on_topic(uxrSession *session, uxrObjectId object_id, uint16_t request_id,
    uxrStreamId stream_id, struct ucdrBuffer *ub, uint16_t length, void *args)
{
  (void)session; (void)request_id; (void)stream_id; (void)length;
  ros2node_data_t *data = (ros2node_data_t *)args;
  mrb_state *mrb = data->mrb;

  char buf[256];
  if (!ucdr_deserialize_string(ub, buf, sizeof(buf))) return;

  mrb_value pending = mrb_iv_get(mrb, data->self, MRB_IVSYM(pending));
  if (!mrb_array_p(pending)) return;

  mrb_value pair = mrb_ary_new_capa(mrb, 2);
  mrb_ary_push(mrb, pair, mrb_fixnum_value(((mrb_int)object_id.id << 8) | object_id.type));
  mrb_ary_push(mrb, pair, mrb_str_new_cstr(mrb, buf));
  mrb_ary_push(mrb, pending, pair);
}

/* Session keys must be unique per client at the agent; derive one from the
 * node name (FNV-1a) instead of requiring the caller to pick one. */
static uint32_t
ros2node_key_from_name(const char *name, mrb_int len)
{
  uint32_t hash = 2166136261u;
  for (mrb_int i = 0; i < len; i++) {
    hash ^= (uint8_t)name[i];
    hash *= 16777619u;
  }
  return hash == 0 ? 1 : hash;
}

static mrb_value
mrb_ros2node_initialize(mrb_state *mrb, mrb_value self)
{
  mrb_value name, io;
  mrb_get_args(mrb, "So", &name, &io);

  /* recvfrom_nonblock is UDPSocket-only; check it first. UDPSocket also
   * defines readpartial (it just raises NotImplementedError), so checking
   * that first would misdetect every UDPSocket as a UART. */
  ros2node_io_kind io_kind;
  if (mrb_respond_to(mrb, io, MRB_SYM(recvfrom_nonblock))) {
    io_kind = ROS2NODE_IO_UDP;
  } else if (mrb_respond_to(mrb, io, MRB_SYM(readpartial))) {
    io_kind = ROS2NODE_IO_UART;
  } else {
    mrb_raise(mrb, E_ARGUMENT_ERROR, "ROS2::Node: io must be a UART or a connected UDPSocket");
  }

  ros2node_data_t *data = (ros2node_data_t *)mrb_malloc(mrb, sizeof(ros2node_data_t));
  memset(data, 0, sizeof(ros2node_data_t));
  data->mrb = mrb;
  data->self = self;
  data->io = io;
  data->io_kind = io_kind;

  DATA_PTR(self) = data;
  DATA_TYPE(self) = &ros2node_data_type;

  /* Keeps io reachable for the GC independently of our own struct. */
  mrb_iv_set(mrb, self, MRB_IVSYM(io), io);
  /* Drained by #spin_once, filled by ros2node_on_topic; see its comment. */
  mrb_iv_set(mrb, self, MRB_IVSYM(pending), mrb_ary_new(mrb));

  /* Byte streams (UART) need COBS-style framing to find message
   * boundaries; datagrams (UDP) are already one message per packet. */
  bool framing = io_kind == ROS2NODE_IO_UART;
  uxr_set_custom_transport_callbacks(&data->transport, framing,
      ros2node_transport_open, ros2node_transport_close,
      ros2node_transport_write, ros2node_transport_read);
  if (!uxr_init_custom_transport(&data->transport, data)) {
    mrb_raise(mrb, E_RUNTIME_ERROR, "ROS2::Node: failed to init transport");
  }

  uint32_t key = ros2node_key_from_name(RSTRING_PTR(name), RSTRING_LEN(name));
  uxr_init_session(&data->session, &data->transport.comm, key);
  uxr_set_topic_callback(&data->session, ros2node_on_topic, data);
  if (!uxr_create_session(&data->session)) {
    mrb_raise(mrb, E_RUNTIME_ERROR, "ROS2::Node: failed to create session with agent");
  }

  data->reliable_out = uxr_create_output_reliable_stream(&data->session, data->output_buffer,
      sizeof(data->output_buffer), ROS2NODE_STREAM_HISTORY);
  data->reliable_in = uxr_create_input_reliable_stream(&data->session, data->input_buffer,
      sizeof(data->input_buffer), ROS2NODE_STREAM_HISTORY);

  /* A Node is one DDS Participant with one Publisher shared by every topic
   * it ends up publishing to (id 1 in both namespaces); each publish()'d
   * topic then gets its own Topic+DataWriter pair, allocated from
   * next_object_id starting at 2. Created eagerly here (rather than lazily
   * on first publish) since every Node needs a participant regardless. */
  data->next_object_id = 2;
  data->participant_id = uxr_object_id(1, UXR_PARTICIPANT_ID);
  data->publisher_id = uxr_object_id(1, UXR_PUBLISHER_ID);

  uint16_t participant_req = uxr_buffer_create_participant_bin(&data->session, data->reliable_out,
      data->participant_id, 0, mrb_string_value_cstr(mrb, &name), UXR_REPLACE);
  uint16_t publisher_req = uxr_buffer_create_publisher_bin(&data->session, data->reliable_out,
      data->publisher_id, data->participant_id, UXR_REPLACE);

  uint16_t requests[2] = { participant_req, publisher_req };
  uint8_t status[2];
  if (!uxr_run_session_until_all_status(&data->session, 1000, requests, status, 2)) {
    mrb_raise(mrb, E_RUNTIME_ERROR, "ROS2::Node: failed to create participant/publisher");
  }

  return self;
}

/* topic_name/type_name are the DDS-wire names (e.g. "rt/chatter",
 * "std_msgs::msg::dds_::String_") -- mrblib is responsible for the ROS2
 * name-mangling, this layer just creates the entities. Returns the
 * DataWriter's uxrObjectId packed into a Fixnum ((id << 8) | type) for
 * mrblib to hold onto and pass back into #_write_string et al. */
static mrb_value
mrb_ros2node_create_publisher(mrb_state *mrb, mrb_value self)
{
  ros2node_data_t *data = (ros2node_data_t *)mrb_data_get_ptr(mrb, self, &ros2node_data_type);
  mrb_value topic_name, type_name;
  mrb_get_args(mrb, "SS", &topic_name, &type_name);

  uint16_t n = data->next_object_id++;
  uxrObjectId topic_id = uxr_object_id(n, UXR_TOPIC_ID);
  uxrObjectId datawriter_id = uxr_object_id(n, UXR_DATAWRITER_ID);

  uint16_t topic_req = uxr_buffer_create_topic_bin(&data->session, data->reliable_out, topic_id,
      data->participant_id, mrb_string_value_cstr(mrb, &topic_name), mrb_string_value_cstr(mrb, &type_name),
      UXR_REPLACE);

  /* Matches rclcpp's default publisher QoS (reliable/volatile/keep_last-10)
   * so a plain `ros2 topic echo` subscriber is compatible out of the box. */
  uxrQoS_t qos = {
    .durability = UXR_DURABILITY_VOLATILE,
    .reliability = UXR_RELIABILITY_RELIABLE,
    .history = UXR_HISTORY_KEEP_LAST,
    .depth = 10,
  };
  uint16_t datawriter_req = uxr_buffer_create_datawriter_bin(&data->session, data->reliable_out, datawriter_id,
      data->publisher_id, topic_id, qos, UXR_REPLACE);

  uint16_t requests[2] = { topic_req, datawriter_req };
  uint8_t status[2];
  if (!uxr_run_session_until_all_status(&data->session, 1000, requests, status, 2)) {
    mrb_raise(mrb, E_RUNTIME_ERROR, "ROS2::Node: failed to create publisher");
  }

  return mrb_fixnum_value(((mrb_int)datawriter_id.id << 8) | datawriter_id.type);
}

/* Serializes str as a CDR string (uint32 length prefix + bytes + NUL) and
 * writes it to the DataWriter identified by the packed id #_create_publisher
 * returned. Best-effort from our side too: send now, don't wait for the
 * agent's delivery ack. #spin_once now exists and pumps the reliable
 * stream's heartbeats/acks when called, but publish() itself still doesn't
 * block on delivery confirmation -- see Risks in the tracking issue for
 * the sustained-pub/sub-traffic caveat this implies. */
static mrb_value
mrb_ros2node_write_string(mrb_state *mrb, mrb_value self)
{
  ros2node_data_t *data = (ros2node_data_t *)mrb_data_get_ptr(mrb, self, &ros2node_data_type);
  mrb_int packed;
  mrb_value str;
  mrb_get_args(mrb, "iS", &packed, &str);

  uxrObjectId datawriter_id = uxr_object_id((uint16_t)(packed >> 8), (uint8_t)(packed & 0xFF));

  ucdrBuffer ub;
  uint32_t size = 4 + (uint32_t)RSTRING_LEN(str) + 1;
  uxr_prepare_output_stream(&data->session, data->reliable_out, datawriter_id, &ub, size);
  ucdr_serialize_string(&ub, mrb_string_value_cstr(mrb, &str));
  uxr_flash_output_streams(&data->session);

  return ub.error ? mrb_false_value() : mrb_true_value();
}

/* topic_name/type_name are the DDS-wire names, same convention as
 * #_create_publisher. The Subscriber entity (id 1, like the Publisher) is
 * created lazily here on first use rather than eagerly in #initialize,
 * since a publish-only Node has no need for one. Requests data delivery
 * immediately (unlimited samples, no rate limiting -- fine for this MVP's
 * single low-rate String topic) so ros2node_on_topic starts firing once
 * #spin_once is called. Returns the DataReader's packed id, same scheme
 * as #_create_publisher's DataWriter id. */
static mrb_value
mrb_ros2node_create_subscriber(mrb_state *mrb, mrb_value self)
{
  ros2node_data_t *data = (ros2node_data_t *)mrb_data_get_ptr(mrb, self, &ros2node_data_type);
  mrb_value topic_name, type_name;
  mrb_get_args(mrb, "SS", &topic_name, &type_name);

  if (!data->has_subscriber) {
    data->subscriber_id = uxr_object_id(1, UXR_SUBSCRIBER_ID);
    uint16_t subscriber_req = uxr_buffer_create_subscriber_bin(&data->session, data->reliable_out,
        data->subscriber_id, data->participant_id, UXR_REPLACE);
    uint8_t status;
    if (!uxr_run_session_until_all_status(&data->session, 1000, &subscriber_req, &status, 1)) {
      mrb_raise(mrb, E_RUNTIME_ERROR, "ROS2::Node: failed to create subscriber");
    }
    data->has_subscriber = true;
  }

  uint16_t n = data->next_object_id++;
  uxrObjectId topic_id = uxr_object_id(n, UXR_TOPIC_ID);
  uxrObjectId datareader_id = uxr_object_id(n, UXR_DATAREADER_ID);

  uint16_t topic_req = uxr_buffer_create_topic_bin(&data->session, data->reliable_out, topic_id,
      data->participant_id, mrb_string_value_cstr(mrb, &topic_name), mrb_string_value_cstr(mrb, &type_name),
      UXR_REPLACE);

  uxrQoS_t qos = {
    .durability = UXR_DURABILITY_VOLATILE,
    .reliability = UXR_RELIABILITY_RELIABLE,
    .history = UXR_HISTORY_KEEP_LAST,
    .depth = 10,
  };
  uint16_t datareader_req = uxr_buffer_create_datareader_bin(&data->session, data->reliable_out, datareader_id,
      data->subscriber_id, topic_id, qos, UXR_REPLACE);

  uint16_t requests[2] = { topic_req, datareader_req };
  uint8_t status2[2];
  if (!uxr_run_session_until_all_status(&data->session, 1000, requests, status2, 2)) {
    mrb_raise(mrb, E_RUNTIME_ERROR, "ROS2::Node: failed to create subscriber's topic/datareader");
  }

  uxrDeliveryControl delivery_control = { 0 };
  delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
  uxr_buffer_request_data(&data->session, data->reliable_out, datareader_id, data->reliable_in, &delivery_control);
  uxr_flash_output_streams(&data->session);

  return mrb_fixnum_value(((mrb_int)datareader_id.id << 8) | datareader_id.type);
}

/* Pumps the transport for up to timeout_ms: sends any buffered output,
 * processes incoming messages (heartbeats/acks for the reliable stream,
 * and DATA submessages -> ros2node_on_topic, which just queues them; see
 * its comment for why). Once back here -- safely outside
 * Micro-XRCE-DDS-Client's call stack -- drains that queue and invokes the
 * matching #subscribe block for each entry via mrblib's #_dispatch_pending. */
static mrb_value
mrb_ros2node_spin_once(mrb_state *mrb, mrb_value self)
{
  ros2node_data_t *data = (ros2node_data_t *)mrb_data_get_ptr(mrb, self, &ros2node_data_type);
  mrb_int timeout_ms = 0;
  mrb_get_args(mrb, "|i", &timeout_ms);

  uxr_run_session_timeout(&data->session, (int)timeout_ms);
  mrb_funcall(mrb, self, "_dispatch_pending", 0);

  return self;
}

void
mrb_picoruby_ros2node_gem_init(mrb_state* mrb)
{
  struct RClass *module_ROS2 = mrb_define_module_id(mrb, MRB_SYM(ROS2));
  struct RClass *class_Node = mrb_define_class_under_id(mrb, module_ROS2, MRB_SYM(Node), mrb->object_class);
  MRB_SET_INSTANCE_TT(class_Node, MRB_TT_CDATA);

  mrb_define_method_id(mrb, class_Node, MRB_SYM(initialize), mrb_ros2node_initialize, MRB_ARGS_REQ(2));
  mrb_define_method_id(mrb, class_Node, MRB_SYM(_create_publisher), mrb_ros2node_create_publisher, MRB_ARGS_REQ(2));
  mrb_define_method_id(mrb, class_Node, MRB_SYM(_write_string), mrb_ros2node_write_string, MRB_ARGS_REQ(2));
  mrb_define_method_id(mrb, class_Node, MRB_SYM(_create_subscriber), mrb_ros2node_create_subscriber, MRB_ARGS_REQ(2));
  mrb_define_method_id(mrb, class_Node, MRB_SYM(spin_once), mrb_ros2node_spin_once, MRB_ARGS_OPT(1));
}

void
mrb_picoruby_ros2node_gem_final(mrb_state* mrb)
{
}
