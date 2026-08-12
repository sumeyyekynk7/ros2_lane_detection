import rclpy
from rclpy.node import Node
import cv2
import os
import numpy as np


class LaneNode(Node):

    def __init__(self):
        super().__init__('lane_node')

        self.get_logger().info('Lane detection node started.')

        image_path = os.path.expanduser(
            '~/ros2_ws/src/lane_detection_pkg/images/road.jpg'
        )

        image = cv2.imread(image_path)

        if image is None:
            self.get_logger().error('Image could not be loaded!')
            return

        height, width, channels = image.shape

        self.get_logger().info(
            f'Image loaded: width={width}, height={height}, channels={channels}'
        )

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        edges = cv2.Canny(gray, 100, 200)

        height, width = edges.shape

        mask = np.zeros_like(edges)

        points = np.array([
            [
                (0, height),
                (width // 2, int(height * 0.55)),
                (width, height)
            ]
        ], np.int32)

        cv2.fillPoly(mask, points, 255)
        roi = cv2.bitwise_and(edges, mask)
            

        cv2.imshow('Original', image)
        cv2.imshow('Grayscale', gray)
        cv2.imshow('Edges', edges)
        cv2.imshow('Mask', mask)
        cv2.imshow('ROI', roi)

        cv2.waitKey(0)
        cv2.destroyAllWindows()


def main(args=None):

    rclpy.init(args=args)

    node = LaneNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()