import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np
from tf2_ros import TransformException, Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker

class HumanDetector(Node):
    def __init__(self):
        super().__init__('human_detector')

        # Subscribers
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publishers
        self.point_publisher = self.create_publisher(PointStamped, '/human_position', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/human_pose', 10)
        self.marker_publisher = self.create_publisher(Marker, '/human_marker', 10)

        # Adjustable thresholds
        self.range_diff_threshold = 0.4  # meters, gap between objects
        self.min_leg_width = 0.08        # meters
        self.max_leg_width = 0.2         # meters
        self.max_leg_dist = 0.4          # meters, distance between legs

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def scan_callback(self, msg):
        # Replace inf/nan values in ranges
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges) | np.isnan(ranges)] = 10.0

        # Convert to Cartesian coordinates
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        points = np.column_stack((x, y))

        # Split into segments
        segments = self.split_into_segments(points)
        if not segments:
            return

        self.get_logger().info(f"Found {len(segments)} segments")

        # Identify potential legs
        legs = self.classify_legs(segments)
        if not legs:
            return

        self.get_logger().info(f"Found {len(legs)} potential legs")

        # Detect human based on leg pairs
        human_pos = self.find_human(legs)
        if human_pos is not None:
            # Create a PointStamped in laser frame
            point_laser = PointStamped()
            point_laser.header.frame_id = "laser"
            point_laser.header.stamp = self.get_clock().now().to_msg()
            point_laser.point.x = float(human_pos[0])
            point_laser.point.y = float(human_pos[1])
            point_laser.point.z = 0.0

            try:
                # Transform to base_link frame
                transform = self.tf_buffer.lookup_transform("base_link", "laser", rclpy.time.Time())
                point_base = do_transform_point(point_laser, transform)

                # Publish PoseStamped in base_link frame
                pose_msg = PoseStamped()
                pose_msg.header = point_base.header
                pose_msg.pose.position = point_base.point
                pose_msg.pose.orientation.w = 1.0  # Identity orientation
                self.pose_publisher.publish(pose_msg)

                # Publish Marker in base_link frame
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = 0  # Unique ID for human marker
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.pose.position = point_base.point
                marker.pose.position.z = 0.5  # Center of a 1m tall cylinder
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.5  # Diameter in x (m)
                marker.scale.y = 0.5  # Diameter in y (m)
                marker.scale.z = 1.0  # Height (m)
                marker.color.a = 1.0  # Alpha (fully opaque)
                marker.color.r = 0.0  # Red
                marker.color.g = 1.0  # Green
                marker.color.b = 0.0  # Blue
                self.marker_publisher.publish(marker)

            except TransformException as e:
                self.get_logger().warn(f"Could not transform point: {e}")
                return
        else:
            # No human detected, delete the marker
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = 0
            marker.action = Marker.DELETE
            self.marker_publisher.publish(marker)

    def split_into_segments(self, points):
        """ Splits point cloud data into segments based on distance gaps. """
        segments = []
        current_segment = [points[0]]

        for i in range(1, len(points)):
            dist = np.linalg.norm(points[i] - points[i - 1])
            if dist > self.range_diff_threshold:
                if len(current_segment) > 1:  # Ignore single-point segments
                    segments.append(np.array(current_segment))
                current_segment = [points[i]]
            else:
                current_segment.append(points[i])

        if len(current_segment) > 1:
            segments.append(np.array(current_segment))

        return segments

    def classify_legs(self, segments):
        """ Classifies segments as potential legs based on width. """
        legs = []
        for seg in segments:
            if len(seg) < 2:
                continue
            width = np.linalg.norm(seg[0] - seg[-1])
            if self.min_leg_width < width < self.max_leg_width:
                legs.append(seg)
        return legs

    def find_human(self, legs):
        """ Identifies human position based on leg pairs. """
        for i in range(len(legs)):
            for j in range(i + 1, len(legs)):
                center_i = np.mean(legs[i], axis=0)
                center_j = np.mean(legs[j], axis=0)
                dist = np.linalg.norm(center_i - center_j)
                if dist < self.max_leg_dist:
                    human_pos = (center_i + center_j) / 2
                    return human_pos
        return None

def main():
    rclpy.init()
    node = HumanDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
