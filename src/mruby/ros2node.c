#include <string.h>

#include <mruby.h>
#include <mruby/data.h>
#include <mruby/string.h>
#include <mruby/variable.h>
#include <mruby/presym.h>
#include <mruby/class.h>
#include <mruby/error.h>

#include <uxr/client/client.h>
#include <uxr/client/util/time.h>

typedef struct {
  mrb_state *mrb;
  mrb_value uart;
  uxrCustomTransport transport;
  uxrSession session;
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
 * The UART is opened by the caller before it reaches us, so transport
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
  mrb_value str = mrb_str_new(data->mrb, (const char *)buf, len);
  mrb_value written = mrb_funcall(data->mrb, data->uart, "write", 1, str);
  *error_code = 0;
  return mrb_integer_p(written) ? (size_t)mrb_integer(written) : 0;
}

/* UART#readpartial is non-blocking, so the timeout is enforced here by
 * polling it against uxr_millis() rather than by a blocking driver call. */
static size_t
ros2node_transport_read(uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout_ms, uint8_t *error_code)
{
  ros2node_data_t *data = (ros2node_data_t *)transport->args;
  mrb_state *mrb = data->mrb;
  int64_t deadline = uxr_millis() + timeout_ms;

  for (;;) {
    mrb_value chunk = mrb_funcall(mrb, data->uart, "readpartial", 1, mrb_fixnum_value((mrb_int)len));
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
    mrb_funcall(mrb, data->uart, "sleep_ms", 1, mrb_fixnum_value(1));
  }
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
  mrb_value name, uart;
  mrb_get_args(mrb, "So", &name, &uart);

  ros2node_data_t *data = (ros2node_data_t *)mrb_malloc(mrb, sizeof(ros2node_data_t));
  memset(data, 0, sizeof(ros2node_data_t));
  data->mrb = mrb;
  data->uart = uart;

  DATA_PTR(self) = data;
  DATA_TYPE(self) = &ros2node_data_type;

  /* Keeps uart reachable for the GC independently of our own struct. */
  mrb_iv_set(mrb, self, MRB_IVSYM(uart), uart);

  uxr_set_custom_transport_callbacks(&data->transport, true,
      ros2node_transport_open, ros2node_transport_close,
      ros2node_transport_write, ros2node_transport_read);
  if (!uxr_init_custom_transport(&data->transport, data)) {
    mrb_raise(mrb, E_RUNTIME_ERROR, "ROS2::Node: failed to init transport");
  }

  uint32_t key = ros2node_key_from_name(RSTRING_PTR(name), RSTRING_LEN(name));
  uxr_init_session(&data->session, &data->transport.comm, key);
  if (!uxr_create_session(&data->session)) {
    mrb_raise(mrb, E_RUNTIME_ERROR, "ROS2::Node: failed to create session with agent");
  }

  return self;
}

void
mrb_picoruby_ros2node_gem_init(mrb_state* mrb)
{
  struct RClass *module_ROS2 = mrb_define_module_id(mrb, MRB_SYM(ROS2));
  struct RClass *class_Node = mrb_define_class_under_id(mrb, module_ROS2, MRB_SYM(Node), mrb->object_class);
  MRB_SET_INSTANCE_TT(class_Node, MRB_TT_CDATA);

  mrb_define_method_id(mrb, class_Node, MRB_SYM(initialize), mrb_ros2node_initialize, MRB_ARGS_REQ(2));
}

void
mrb_picoruby_ros2node_gem_final(mrb_state* mrb)
{
}
