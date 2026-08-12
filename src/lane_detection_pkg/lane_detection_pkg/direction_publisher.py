import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class DirectionPublisher(Node):

    def __init__(self):
        super().__init__('direction_publisher')

        self.publisher = self.create_publisher(
            Float32,
            'lane_offset',
            10
        )
        self.offsets = [-25.0, 0.0, 25.0]
        self.offset_index = 0
        self.timer = self.create_timer(1.0, self.publish_offset)

    def publish_offset(self):
        message = Float32()
        message.data = self.offsets[self.offset_index]
        self.offset_index = (self.offset_index + 1) % len(self.offsets)

        self.publisher.publish(message)
        self.get_logger().info(f'Published lane offset: {message.data}')


def main(args=None):
    rclpy.init(args=args)
    node = DirectionPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
