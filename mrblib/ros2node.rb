module ROS2
  class Node
    # cdr:      which #_write_* method serializes this class
    # dds_type: the ROS2/DDS-mangled type name, e.g. rosidl_typesupport_fastrtps
    #           generates "<pkg>::msg::dds_::<Type>_" -- required for a plain
    #           `ros2 topic echo` to recognize the topic as that message type.
    TYPE_MAP = {
      String => { cdr: :string, dds_type: 'std_msgs::msg::dds_::String_' }
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

    def subscribe(topic, type, &block)
      # TODO: lazily create/cache subscriber for topic, register block
    end

    def spin_once(timeout_ms = 0)
      # TODO: non-blocking spin, invoke subscribe blocks on receipt
    end
  end
end
