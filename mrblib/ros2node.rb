module ROS2
  class Node
    TYPE_MAP = {
      String => :string
    }

    def initialize(name, uart)
      # TODO: build transport from uart, uxr_create_session, create participant
    end

    def publish(topic, msg)
      type = TYPE_MAP[msg.class]
      raise NotImplementedError, "unsupported message class: #{msg.class}" unless type
      # TODO: lazily create/cache publisher for topic, then publish
    end

    def subscribe(topic, type, &block)
      # TODO: lazily create/cache subscriber for topic, register block
    end

    def spin_once(timeout_ms = 0)
      # TODO: non-blocking spin, invoke subscribe blocks on receipt
    end
  end
end
