import rclpy
from rclpy.node import Node


class LaneNode(Node):

    def __init__(self):
        super().__init__('lane_node')
        self.get_logger().info('Lane detection node started.')


def main(args=None):
    rclpy.init(args=args)

    node = LaneNode()
    
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
