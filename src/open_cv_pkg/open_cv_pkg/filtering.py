import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Header, Float64MultiArray
from geometry_msgs.msg import Quaternion, TransformStamped
import pcl  # Requires python-pcl
import time  # For delays
from tf2_ros import TransformBroadcaster
from transforms3d.quaternions import quat2mat, mat2quat  # Ensure mat2quat is available

class PointCloudFilteringNode(Node):
    def __init__(self):
        super().__init__('pointcloud_filtering_node')

        # Localization: storage for latest transformation data
        self.latest_matrix = None
        self.latest_quaternion = None

        # Dynamic Transform Broadcaster (for publishing realtime TF transforms)
        self.br = TransformBroadcaster(self)

        # Subscribe to localization topics
        self.create_subscription(Float64MultiArray, "/localization_matrix", self.matrix_callback, 10)
        self.create_subscription(Quaternion, "/localization_quaternion", self.quaternion_callback, 10)

        # Publishers for various stages
        self.downsampled_pub = self.create_publisher(PointCloud2, '/downsampled_points', 10)
        self.transformed_pub = self.create_publisher(PointCloud2, '/transformed_points', 10)
        self.ground_pub = self.create_publisher(PointCloud2, '/ground_points', 10)
        self.non_ground_pub = self.create_publisher(PointCloud2, '/non_ground_points', 10)
        self.wall_pub = self.create_publisher(PointCloud2, '/wall_points', 10)

        # Subscriber to raw point cloud from camera
        self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',  # Adjust if necessary
            self.pointcloud_callback,
            10
        )

        # (Optional) TF Buffer and Listener are still created in case you need them later
        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Rate limiting parameters for logging
        self.last_log_time = time.time()
        self.log_interval = 3.0  # seconds

        self.get_logger().info('PointCloud Filtering Node started.')

    def should_log(self):
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            self.last_log_time = current_time
            return True
        return False

    # Callback to load the localization matrix
    def matrix_callback(self, msg):
        if len(msg.data) == 16:
            self.latest_matrix = np.array(msg.data, dtype=np.float64).reshape((4, 4))
            self.get_logger().info("Localization matrix updated.")
            self.publish_transform()  # Publish a new TF transform based on the updated matrix

    # Callback to load the localization quaternion (currently not used in the transform, but stored)
    def quaternion_callback(self, msg):
        self.latest_quaternion = np.array([msg.w, msg.x, msg.y, msg.z], dtype=np.float64)
        self.get_logger().info("Localization quaternion updated.")

    def publish_transform(self):
        """
        Publish a dynamic TF transform from "map" to "base_link" using the latest localization matrix.
        """
        if self.latest_matrix is None:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"          # Global frame
        t.child_frame_id = "base_link"       # Rover's frame

        # Translation: elements of the last column (first three rows)
        t.transform.translation.x = float(self.latest_matrix[0, 3])
        t.transform.translation.y = float(self.latest_matrix[1, 3])
        t.transform.translation.z = float(self.latest_matrix[2, 3])

        # Rotation: convert the 3x3 rotation matrix to a quaternion.
        quat = mat2quat(self.latest_matrix[:3, :3])
        # Note: transforms3d.mat2quat returns (w, x, y, z). geometry_msgs expects (x, y, z, w)
        t.transform.rotation.x = quat[1]
        t.transform.rotation.y = quat[2]
        t.transform.rotation.z = quat[3]
        t.transform.rotation.w = quat[0]

        self.br.sendTransform(t)
        self.get_logger().info("Published dynamic transform from map to base_link.")

    def pointcloud_callback(self, msg):
        """Process and filter point cloud data."""
        # Convert the raw PointCloud2 message into a NumPy array and downsample it
        raw_points = self.read_pointcloud(msg)
        downsampled_points = self.downsample_pointcloud(raw_points, leaf_size=0.1)

        # Transform the downsampled point cloud using the localization matrix
        transformed_points = self.transform_pointcloud(downsampled_points, msg.header.frame_id)

        # Pre-filter: remove points above a certain height and points too far away
        filtered_points = self.filter_high_points(transformed_points, max_height=0.4)
        filtered_points = self.filter_far_points(filtered_points, max_depth=3.0)

        # Segment ground from non-ground points using RANSAC
        ground_points, non_ground_points = self.segment_ground(filtered_points)

        # Further segment vertical wall planes from the non-ground points using RANSAC.
        wall_points, non_wall_points = self.segment_walls(non_ground_points, normal_threshold=0.3, distance_threshold=0.065)

        # Apply Euclidean clustering on non-wall points with relaxed cluster size filtering.
        clustered_obstacle_points = self.apply_clustering(non_wall_points)

        # Publish all results using "base_link" as the frame
        self.publish_pointcloud(downsampled_points, self.downsampled_pub, "base_link")
        self.publish_pointcloud(transformed_points, self.transformed_pub, "base_link")
        self.publish_pointcloud(ground_points, self.ground_pub, "base_link")
        self.publish_pointcloud(clustered_obstacle_points, self.non_ground_pub, "base_link")
        self.publish_pointcloud(wall_points, self.wall_pub, "base_link")

    def read_pointcloud(self, msg):
        """Convert the raw PointCloud2 message into a NumPy array."""
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        return np.array([(p[0], p[1], p[2]) for p in points], dtype=np.float32)

    def downsample_pointcloud(self, points, leaf_size=0.1):
        """Downsample the point cloud using a voxel grid filter."""
        if len(points) == 0:
            return points
        
        cloud = pcl.PointCloud(points.astype(np.float32))
        voxel_filter = cloud.make_voxel_grid_filter()
        voxel_filter.set_leaf_size(leaf_size, leaf_size, leaf_size)
        downsampled_cloud = voxel_filter.filter()
        downsampled_array = np.asarray(downsampled_cloud)

        if self.should_log():
            self.get_logger().info(f"Downsampled to {len(downsampled_array)} points (leaf size: {leaf_size}).")
        return downsampled_array

    def transform_pointcloud(self, points, source_frame):
        """Transform the point cloud using the localization matrix."""
        if self.latest_matrix is None:
            if self.should_log():
                self.get_logger().warn("No localization matrix received yet.")
            return points  # Return the original points if no matrix is available

        try:
            # Extract rotation (3x3) and translation (3x1) from the 4x4 matrix
            rotation_matrix = self.latest_matrix[:3, :3]
            translation_vector = self.latest_matrix[:3, 3]
            # Apply the transformation: p' = R * p + t for each point
            return (points @ rotation_matrix.T) + translation_vector
        except Exception as e:
            self.get_logger().error(f"Failed to apply localization matrix: {e}")
            return points

    def filter_high_points(self, points, max_height):
        """Filter out points above a specified height."""
        return points[points[:, 2] < max_height]

    def filter_far_points(self, points, max_depth):
        """Filter out points beyond a specified depth (x-axis)."""
        filtered_points = points[points[:, 0] < max_depth]
        if self.should_log():
            self.get_logger().info(f"Filtered far points: {len(filtered_points)} remaining within {max_depth}m.")
        return filtered_points

    def segment_ground(self, points):
        """Segment the ground plane using RANSAC."""
        if len(points) == 0:
            return np.empty((0, 3)), np.empty((0, 3))
        
        cloud = pcl.PointCloud(points.astype(np.float32))
        seg = cloud.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.05)
        indices, coefficients = seg.segment()
        ground_points = points[indices] if indices else np.empty((0, 3))
        non_ground_points = np.delete(points, indices, axis=0) if indices else points

        if self.should_log():
            self.get_logger().info(f"Segmented ground: {len(ground_points)} points, {len(non_ground_points)} non-ground points.")
        return ground_points, non_ground_points

    def segment_walls(self, points, normal_threshold=0.6, distance_threshold=0.3):
        """
        Segment vertical wall planes from non-ground points using RANSAC.
        """
        if len(points) == 0:
            return np.empty((0, 3)), points
        
        cloud = pcl.PointCloud(points.astype(np.float32))
        seg = cloud.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(distance_threshold)
        indices, coefficients = seg.segment()
        if not indices:
            return np.empty((0, 3)), points

        if abs(coefficients[2]) < normal_threshold:
            wall_points = points[indices]
            remaining_points = np.delete(points, indices, axis=0)
            if self.should_log():
                self.get_logger().info(f"Detected {len(wall_points)} wall points (|c| = {abs(coefficients[2]):.3f}).")
            return wall_points, remaining_points
        else:
            return np.empty((0, 3)), points

    def apply_clustering(self, points):
        """
        Apply Euclidean clustering on the given points and filter clusters based on size.
        """
        if len(points) == 0:
            return points

        cloud = pcl.PointCloud(points.astype(np.float32))
        tree = cloud.make_kdtree()
        ec = cloud.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.1)
        ec.set_MinClusterSize(15)
        ec.set_MaxClusterSize(500)
        ec.set_SearchMethod(tree)
        cluster_indices = ec.Extract()
        filtered_points = []
        for indices in cluster_indices:
            if len(indices) >= 15 and len(indices) <= 500:
                for idx in indices:
                    filtered_points.append(points[idx])
        return np.array(filtered_points, dtype=np.float32)

    def publish_pointcloud(self, points, publisher, frame_id):
        """Publish a point cloud as a PointCloud2 message."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        msg = pc2.create_cloud_xyz32(header, points.tolist())
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilteringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
