#localization testing
#For localization testing:
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float64
import csv
import math
from collections import deque
from datetime import datetime

class PoseLogger(Node):
    def __init__(self):
        super().__init__('pose_logger')

        # Buffer of last 10 PoseStamped messages
        self.pose_buffer = deque(maxlen=10)

        # Subscriptions
        self.create_subscription(PoseStamped, '/rover_pose', self.pose_callback, 10)
        self.create_subscription(Point, '/log_xy', self.log_xy_callback, 10)
        self.create_subscription(Float64, '/log_distance', self.log_distance_callback, 10)
        self.create_subscription(Point, '/log_xy_heading', self.log_xy_heading_callback, 10)

        self.get_logger().info("PoseLogger node started and ready to log!")

    def pose_callback(self, msg):
        """Stores the latest PoseStamped message."""
        self.pose_buffer.append(msg)

    def average_pose(self):
        """Computes average x, y, z and yaw (heading) from the last 10 poses."""
        if len(self.pose_buffer) < 10:
            self.get_logger().warn("Not enough data to average (need 10 poses).")
            return None

        sum_x = sum_y = sum_z = 0.0
        sum_yaw = 0.0

        for pose in self.pose_buffer:
            pos = pose.pose.position
            sum_x += pos.x
            sum_y += pos.y
            sum_z += pos.z

            # Compute yaw from quaternion
            q = pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
            sum_yaw += yaw

        avg_x = sum_x / 10
        avg_y = sum_y / 10
        avg_z = sum_z / 10
        avg_yaw = sum_yaw / 10

        return (avg_x, avg_y, avg_z, avg_yaw)

    def log_xy_callback(self, msg: Point):
        """Logs manual x, y input + averaged x, y to a CSV."""
        avg = self.average_pose()
        if avg is None:
            return
        avg_x, avg_y, _, _ = avg
        filename = "xy_log.csv"
        with open(filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([msg.x, msg.y, avg_x, avg_y, datetime.now()])
        self.get_logger().info(f"Logged XY to {filename}")

    def log_distance_callback(self, msg: Float64):
        """Logs measured distance + computed distance from origin of average pose."""
        avg = self.average_pose()
        if avg is None:
            return
        avg_x, avg_y, avg_z, _ = avg
        computed_dist = math.sqrt(avg_x**2 + avg_y**2 + avg_z**2)
        filename = "distance_log.csv"
        with open(filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([msg.data, computed_dist, datetime.now()])
        self.get_logger().info(f"Logged Distance to {filename}")

    def log_xy_heading_callback(self, msg: Point):
        """Logs manual x, y input + average x, y, heading to CSV."""
        avg = self.average_pose()
        if avg is None:
            return
        avg_x, avg_y, _, avg_yaw = avg
        filename = "xy_heading_log.csv"
        with open(filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([msg.x, msg.y, avg_x, avg_y, avg_yaw, datetime.now()])
        self.get_logger().info(f"Logged XY Heading to {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()