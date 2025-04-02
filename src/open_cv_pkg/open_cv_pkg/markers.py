import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np


class MarkerNode(Node):
    def __init__(self):
        super().__init__('marker_node')

        # Publishers for various markers
        self.debug_marker_pub = self.create_publisher(MarkerArray, '/debug_markers', 10)
        self.camera_marker_pub = self.create_publisher(Marker, '/camera_marker', 10)
        self.fov_pub = self.create_publisher(Marker, '/camera_fov', 10)
        self.threshold_marker_pub = self.create_publisher(Marker, '/threshold_marker', 10)

        self.get_logger().info("MarkerNode has been started.")

        # Example data for marker publishing
        self.example_points = [
            (0.5, 0.5, 0.5),
            (1.0, 1.0, 1.0),
            (1.5, 1.5, 1.5)
        ]

        # Threshold bounds for the threshold marker
        self.x_min, self.x_max = -5.0, 5.0
        self.y_min, self.y_max = 0.5, 2.0
        self.z_min, self.z_max = 0.0, 10.0

        # Timer to periodically publish markers
        self.create_timer(1.0, self.publish_all_markers)

    def publish_all_markers(self):
        """Publish all markers periodically."""
        self.publish_debug_markers(self.example_points)
        self.publish_camera_marker()
        self.publish_camera_fov()
        self.publish_threshold_marker()

    def publish_debug_markers(self, points, frame_id="camera_link"):
        """Publish debug markers for filtered points."""
        marker_array = MarkerArray()
        for i, (x, y, z) in enumerate(points):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "debug_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0  # Opacity
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        self.debug_marker_pub.publish(marker_array)

    def publish_camera_marker(self, frame_id="camera_link"):
        """Publish a marker for the camera position."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "camera_marker"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.g = 1.0  # Green
        self.camera_marker_pub.publish(marker)

    def publish_camera_fov(self, frame_id="camera_link", fov_horizontal=69, fov_vertical=42, max_depth=2.0):
        """Publish the camera FoV marker."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "camera_fov"
        marker.id = 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        # Calculate the four corners of the camera FoV
        fov_horizontal = np.radians(fov_horizontal)
        fov_vertical = np.radians(fov_vertical)

        center = [0.0, 0.0, 0.0]
        top_left = [max_depth * np.tan(fov_vertical / 2), max_depth * np.tan(fov_horizontal / 2), max_depth]
        top_right = [-top_left[0], top_left[1], max_depth]
        bottom_left = [top_left[0], -top_left[1], max_depth]
        bottom_right = [-top_left[0], -top_left[1], max_depth]

        points = [
            center, top_left, center, top_right, center, bottom_left, center, bottom_right,
            top_left, top_right, top_right, bottom_right, bottom_right, bottom_left, bottom_left, top_left
        ]

        for p in points:
            point = Point()
            point.x = p[0]
            point.y = p[1]
            point.z = p[2]
            marker.points.append(point)

        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.fov_pub.publish(marker)

    def publish_threshold_marker(self, frame_id="camera_link"):
        """Publish a marker to visualize the filtering thresholds."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "threshold_box"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Set position and dimensions of the marker based on thresholds
        marker.pose.position.x = (self.x_min + self.x_max) / 2.0
        marker.pose.position.y = (self.y_min + self.y_max) / 2.0
        marker.pose.position.z = (self.z_min + self.z_max) / 2.0

        marker.scale.x = self.x_max - self.x_min
        marker.scale.y = self.y_max - self.y_min
        marker.scale.z = self.z_max - self.z_min

        # Set color and transparency
        marker.color.a = 0.5  # Transparency
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.threshold_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
