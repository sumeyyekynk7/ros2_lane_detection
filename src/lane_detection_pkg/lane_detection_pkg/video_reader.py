from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class VideoReader(Node):

    def __init__(self):
        super().__init__('video_reader')

        default_path = Path(__file__).parent / 'videos' / 'road.mp4'
        self.declare_parameter('video_path', str(default_path))
        self.declare_parameter('center_calibration_px', -200.0)
        video_path = self.get_parameter(
            'video_path'
        ).get_parameter_value().string_value
        self.center_calibration = self.get_parameter(
            'center_calibration_px'
        ).get_parameter_value().double_value

        self.capture = cv2.VideoCapture(video_path)
        if not self.capture.isOpened():
            raise RuntimeError(f'Video could not be opened: {video_path}')

        self.offset_publisher = self.create_publisher(Float32, 'lane_offset', 10)
        self.left_line_model = None
        self.right_line_model = None
        self.left_missing_frames = 0
        self.right_missing_frames = 0
        self.line_memory_frames = 45
        fps = self.capture.get(cv2.CAP_PROP_FPS)
        timer_period = 1.0 / fps if fps > 0.0 else 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.show_next_frame)
        self.get_logger().info(f'Video opened: {video_path}')

    def show_next_frame(self):
        success, frame = self.capture.read()

        if not success:
            self.get_logger().info('Video finished.')
            rclpy.shutdown()
            return

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)
        edge_frame = cv2.Canny(blurred_frame, 50, 150)
        road_edges = self.keep_road_region(edge_frame)
        line_frame, lane_offset = self.draw_detected_lines(frame, road_edges)

        if lane_offset is not None:
            offset_message = Float32()
            offset_message.data = float(lane_offset)
            self.offset_publisher.publish(offset_message)

        cv2.imshow('Original Road Video', frame)
        cv2.imshow('Gray Road Video', gray_frame)
        cv2.imshow('Road Edges', edge_frame)
        cv2.imshow('Road Region Edges', road_edges)
        cv2.imshow('Detected Lane Lines', line_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('Video stopped with Q key.')
            rclpy.shutdown()

    @staticmethod
    def keep_road_region(edge_frame):
        height, width = edge_frame.shape
        road_polygon = np.array([[
            (0, int(height * 0.92)),
            (int(width * 0.43), int(height * 0.50)),
            (int(width * 0.57), int(height * 0.50)),
            (width - 1, int(height * 0.92)),
        ]], dtype=np.int32)

        mask = np.zeros_like(edge_frame)
        cv2.fillPoly(mask, road_polygon, 255)
        return cv2.bitwise_and(edge_frame, mask)

    def draw_detected_lines(self, frame, road_edges):
        line_frame = frame.copy()
        left_lines = []
        right_lines = []
        lines = cv2.HoughLinesP(
            road_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=30,
            minLineLength=30,
            maxLineGap=50
        )

        if lines is not None:
            image_center = road_edges.shape[1] / 2
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x1 == x2:
                    continue

                slope = (y2 - y1) / (x2 - x1)
                if abs(slope) < 0.4:
                    continue

                intercept = y1 - slope * x1
                length = np.hypot(x2 - x1, y2 - y1)
                line_center = (x1 + x2) / 2
                if slope < 0 and line_center < image_center:
                    left_lines.append((slope, intercept, length))
                elif slope > 0 and line_center > image_center:
                    right_lines.append((slope, intercept, length))
                else:
                    continue

                cv2.line(
                    line_frame,
                    (x1, y1),
                    (x2, y2),
                    (0, 0, 255),
                    2
                )

        previous_left = self.left_line_model
        previous_right = self.right_line_model

        self.left_line_model, self.left_missing_frames = self.smooth_line(
            left_lines,
            self.left_line_model,
            self.left_missing_frames,
            self.line_memory_frames,
            frame.shape[0]
        )
        self.right_line_model, self.right_missing_frames = self.smooth_line(
            right_lines,
            self.right_line_model,
            self.right_missing_frames,
            self.line_memory_frames,
            frame.shape[0]
        )

        reference_y = frame.shape[0] * 0.75
        if not left_lines and right_lines:
            right_shift = self.model_shift(
                previous_right, self.right_line_model, reference_y
            )
            self.left_line_model = self.shift_model(
                self.left_line_model, right_shift
            )
        elif left_lines and not right_lines:
            left_shift = self.model_shift(
                previous_left, self.left_line_model, reference_y
            )
            self.right_line_model = self.shift_model(
                self.right_line_model, left_shift
            )

        left_x = self.draw_average_line(
            line_frame, self.left_line_model, (255, 0, 0)
        )
        right_x = self.draw_average_line(
            line_frame, self.right_line_model, (0, 255, 0)
        )

        lane_offset = None
        if left_x is not None and right_x is not None:
            image_center = frame.shape[1] // 2
            lane_center = (left_x + right_x) // 2
            raw_offset = lane_center - image_center
            lane_offset = raw_offset - self.center_calibration

            cv2.circle(
                line_frame, (image_center, frame.shape[0] - 20), 8,
                (0, 255, 255), -1
            )
            cv2.circle(
                line_frame, (lane_center, frame.shape[0] - 20), 8,
                (255, 255, 0), -1
            )
            cv2.putText(
                line_frame,
                f'Lane offset: {lane_offset:.0f} px',
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 255),
                2
            )

        return line_frame, lane_offset

    @staticmethod
    def smooth_line(
        line_group, previous_model, missing_frames, max_missing_frames,
        frame_height
    ):
        if not line_group:
            missing_frames += 1
            if missing_frames > max_missing_frames:
                return None, missing_frames
            return previous_model, missing_frames

        weights = [line[2] for line in line_group]
        slope = np.average([line[0] for line in line_group], weights=weights)
        intercept = np.average(
            [line[1] for line in line_group],
            weights=weights
        )
        new_model = (slope, intercept)

        if previous_model is None:
            return new_model, 0

        reference_y = frame_height * 0.75
        previous_x = (
            reference_y - previous_model[1]
        ) / previous_model[0]
        new_x = (reference_y - new_model[1]) / new_model[0]
        if abs(new_x - previous_x) > 80:
            missing_frames += 1
            if missing_frames > max_missing_frames:
                return None, missing_frames
            return previous_model, missing_frames

        smoothing = 0.20
        smooth_slope = (
            smoothing * new_model[0]
            + (1.0 - smoothing) * previous_model[0]
        )
        smooth_intercept = (
            smoothing * new_model[1]
            + (1.0 - smoothing) * previous_model[1]
        )
        return (smooth_slope, smooth_intercept), 0

    @staticmethod
    def model_shift(previous_model, current_model, reference_y):
        if previous_model is None or current_model is None:
            return 0.0

        previous_x = (
            reference_y - previous_model[1]
        ) / previous_model[0]
        current_x = (
            reference_y - current_model[1]
        ) / current_model[0]
        return current_x - previous_x

    @staticmethod
    def shift_model(line_model, horizontal_shift):
        if line_model is None:
            return None

        slope, intercept = line_model
        shifted_intercept = intercept - slope * horizontal_shift
        return slope, shifted_intercept

    @staticmethod
    def draw_average_line(frame, line_model, color):
        if line_model is None:
            return None

        slope, intercept = line_model

        height, width = frame.shape[:2]
        y1 = int(height * 0.90)
        y2 = int(height * 0.50)
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)

        if not (0 <= x1 < width and 0 <= x2 < width):
            return None

        cv2.line(frame, (x1, y1), (x2, y2), color, 8)
        return x1

    def close(self):
        self.capture.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = VideoReader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
