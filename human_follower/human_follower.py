import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_point
import numpy as np

class HumanFollower(Node):
    def __init__(self):
        super().__init__('human_follower')
        # Subscriber to human pose
        self.subscription = self.create_subscription(
            PoseStamped, '/human_pose', self.pose_callback, 10)
        # Action client for Nav2
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # TF2 for transforming poses
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Parameters
        self.safe_distance = 0.05  # meters, distance to maintain behind human
        self.last_goal = None  # Track last sent goal to avoid spamming
        self.max_delta = 1  # meters, max distance between goals

    def pose_callback(self, msg):
        # Extract the position as a PointStamped for transformation
        point_base = PointStamped()
        point_base.header = msg.header  # Already in base_link from human_detector
        point_base.point = msg.pose.position

        # Transform human position from base_link to map frame
        try:
            point_map = self.tf_buffer.transform(point_base, 'map', timeout=rclpy.duration.Duration(seconds=1.0))
        except TransformException as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return

        # Calculate distance to human in map frame
        dx = point_map.point.x
        dy = point_map.point.y
        distance = np.sqrt(dx**2 + dy**2)

        # Adjust goal to maintain safe distance
        if distance > self.safe_distance:
            # Normalize direction vector and scale to safe_distance
            scale = (distance - self.safe_distance) / distance
            goal_x = dx * scale
            goal_y = dy * scale
        else:
            # If too close, don't move (set goal to current position)
            goal_x = 0.0
            goal_y = 0.0

        # Create goal pose in map frame
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.orientation.w = 1.0  # Keep simple orientation for now

        # Only send new goal if significantly different from last goal
        if self.last_goal is None or self.is_goal_different(goal_pose, self.last_goal):
            self.send_goal(goal_pose)
            self.last_goal = goal_pose

    def is_goal_different(self, new_goal, old_goal, threshold=0.1):
        """Check if new goal is significantly different from old goal."""
        dx = new_goal.pose.position.x - old_goal.pose.position.x
        dy = new_goal.pose.position.y - old_goal.pose.position.y
        return np.sqrt(dx**2 + dy**2) > threshold and np.sqrt(dx**2 + dy**2) < self.max_delta

    def send_goal(self, goal_pose):
        """Send navigation goal to Nav2."""
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 action server not available")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.action_client.send_goal_async(goal_msg)
        self.get_logger().info(f"Sent goal to Nav2: x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}")

def main():
    rclpy.init()
    node = HumanFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()