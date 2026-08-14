import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32


class LaneController(Node):

    def __init__(self):
        super().__init__('lane_controller')

        self.declare_parameter('forward_speed', 0.35)
        self.declare_parameter('steering_gain', 0.008)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('detection_timeout', 0.5)

        self.latest_offset = None
        self.last_detection_time = None
        self.was_stopped = True

        self.velocity_publisher = self.create_publisher(
            Twist, '/lane_robot/cmd_vel', 10
        )
        self.offset_subscription = self.create_subscription(
            Float32, 'lane_offset', self.offset_callback, 10
        )
        self.control_timer = self.create_timer(0.05, self.control_loop)

    def offset_callback(self, message):
        self.latest_offset = message.data
        self.last_detection_time = self.get_clock().now()

    def control_loop(self):
        command = Twist()
        timeout = self.get_parameter(
            'detection_timeout'
        ).get_parameter_value().double_value

        if self.last_detection_time is None:
            self.velocity_publisher.publish(command)
            return

        age = (self.get_clock().now() - self.last_detection_time).nanoseconds
        if age / 1e9 > timeout:
            self.velocity_publisher.publish(command)
            if not self.was_stopped:
                self.get_logger().warning('Lane lost; robot stopped.')
                self.was_stopped = True
            return

        speed = self.get_parameter(
            'forward_speed'
        ).get_parameter_value().double_value
        gain = self.get_parameter(
            'steering_gain'
        ).get_parameter_value().double_value
        max_angular = self.get_parameter(
            'max_angular_speed'
        ).get_parameter_value().double_value

        angular_speed = -gain * self.latest_offset
        angular_speed = max(-max_angular, min(max_angular, angular_speed))
        command.linear.x = speed
        command.angular.z = angular_speed
        self.velocity_publisher.publish(command)

        if self.was_stopped:
            self.get_logger().info('Lane found; autonomous drive started.')
            self.was_stopped = False


def main(args=None):
    rclpy.init(args=args)
    node = LaneController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.velocity_publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
