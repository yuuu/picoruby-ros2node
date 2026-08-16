module ROS2
  class Node
    # cdr:      which #_write_* method serializes this class
    # dds_type: the ROS2/DDS-mangled type name, e.g. rosidl_typesupport_fastrtps
    #           generates "<pkg>::msg::dds_::<Type>_" -- required for a plain
    #           `ros2 topic echo` to recognize the topic as that message type.
    TYPE_MAP = {
      String => { cdr: :string, dds_type: 'std_msgs::msg::dds_::String_' }
    }

    # #subscribe's type: must be given explicitly as a Symbol (e.g. :string)
    # since the receiving side can't infer a Ruby class from wire data the
    # way #publish infers dds_type from msg.class. Same dds_type values as
    # TYPE_MAP above, just keyed the other way around.
    SUBSCRIBE_TYPE_MAP = {
      string: 'std_msgs::msg::dds_::String_'
    }

    # #initialize is defined in src/mruby/ros2node.c: it wires the given
    # io (a picoruby-uart UART or a connected picoruby-socket UDPSocket,
    # detected by duck-typing) into a Micro-XRCE-DDS-Client custom
    # transport, establishes the session, and creates the Participant +
    # Publisher entities (also in ros2node.c) that every topic's
    # Topic+DataWriter pair, created below, is scoped under.

    # Publishers are created lazily (and cached in @writers, keyed by the
    # Ruby-side topic name) on first publish to that topic, per the
    # Ruby API design in the tracking issue.
    def publish(topic, msg)
      type = TYPE_MAP[msg.class]
      raise NotImplementedError, "unsupported message class: #{msg.class}" unless type
      @writers ||= {}
      # ROS2's DDS topic-name mangling: a plain "foo" topic is "rt/foo" on the wire.
      writer = @writers[topic] ||= _create_publisher("rt/#{topic}", type[:dds_type])
      case type[:cdr]
      when :string
        _write_string(writer, msg)
      end
    end

    # Subscribers/DataReaders are created lazily (and cached in @readers,
    # keyed by the Ruby-side topic name) on first subscribe to that topic,
    # mirroring #publish/@writers. The block is looked up by the DataReader's
    # packed id from #_dispatch_pending, called by #spin_once (defined in
    # ros2node.c) once it's safe to do so -- see ros2node_on_topic's comment
    # in ros2node.c for why the block isn't just called straight from there.
    def subscribe(topic, type, &block)
      dds_type = SUBSCRIBE_TYPE_MAP[type]
      raise NotImplementedError, "unsupported message type: #{type.inspect}" unless dds_type
      @readers ||= {}
      @callbacks ||= {}
      reader = @readers[topic] ||= _create_subscriber("rt/#{topic}", dds_type)
      @callbacks[reader] = block
    end

    # Opt-in, self-contained alternative to calling #spin_once in a loop by
    # hand: spawns a background Task that does exactly that, so #subscribe
    # callbacks keep firing without the caller giving up its own thread of
    # control. Deliberately does NOT touch R2P2-ESP32's main_task.rb --
    # that file is shared by every R2P2 user regardless of ros2node, and a
    # per-Node Task (mirroring how picoruby-irq spawns its own dispatcher
    # task) needs no central registry of "active nodes" the way hooking
    # main_task.rb's loop would. Idempotent: a second call returns the
    # already-running task instead of spawning another.
    #
    # Note this differs from rclcpp/rclpy's `spin`, which blocks the
    # calling thread forever -- this one returns immediately and does its
    # work in the background task instead.
    def spin(interval_ms = 100)
      node = self # Task blocks run with self == main, not the lexical
                   # self, so #spin_once below must have an explicit
                   # receiver or it would NoMethodError against `main`.
      @spin_task ||= Task.new(name: "ros2node_spin") do
        loop { node.spin_once(interval_ms) }
      end
    end

    # Called by #spin_once (ros2node.c) after each uxr_run_session_timeout;
    # not meant to be called directly.
    def _dispatch_pending
      pending = @pending
      return unless pending
      until pending.empty?
        reader, value = pending.shift
        cb = @callbacks && @callbacks[reader]
        cb.call(value) if cb
      end
    end
  end
end
