import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String


class DirectionSubscriber(Node):

    def __init__(self):
        super().__init__('direction_subscriber')

        self.declare_parameter('straight_tolerance', 30.0)
        self.declare_parameter('turn_threshold', 45.0)
        self.direction = 'STRAIGHT'
        self.last_logged_direction = None
        self.direction_publisher = self.create_publisher(
            String,
            'steering_direction',
            10
        )
        self.subscription = self.create_subscription(
            Float32,
            'lane_offset',
            self.offset_callback,
            10
        )

    def offset_callback(self, message):
        tolerance = self.get_parameter(
            'straight_tolerance'
        ).get_parameter_value().double_value
        turn_threshold = self.get_parameter(
            'turn_threshold'
        ).get_parameter_value().double_value

        if self.direction == 'STRAIGHT':
            if message.data > turn_threshold:
                self.direction = 'RIGHT'
            elif message.data < -turn_threshold:
                self.direction = 'LEFT'
        elif self.direction == 'RIGHT':
            if message.data < -turn_threshold:
                self.direction = 'LEFT'
            elif message.data <= tolerance:
                self.direction = 'STRAIGHT'
        elif self.direction == 'LEFT':
            if message.data > turn_threshold:
                self.direction = 'RIGHT'
            elif message.data >= -tolerance:
                self.direction = 'STRAIGHT'

        direction_message = String()
        direction_message.data = self.direction
        self.direction_publisher.publish(direction_message)
        if self.direction != self.last_logged_direction:
            self.get_logger().info(
                f'Lane offset: {message.data:.2f} -> {self.direction}'
            )
            self.last_logged_direction = self.direction


def main(args=None):
    rclpy.init(args=args)
    node = DirectionSubscriber()

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
