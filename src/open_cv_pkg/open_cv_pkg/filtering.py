#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Header, Float64MultiArray
from geometry_msgs.msg import Quaternion, TransformStamped
import pcl  # Requires python-pcl
import time
from tf2_ros import TransformBroadcaster
from transforms3d.quaternions import quat2mat, mat2quat  # Ensure mat2quat is available

class PointCloudFilteringNode(Node):
    def __init__(self):
        super().__init__('pointcloud_filtering_node')

        # Declare ROS parameters so they can be tuned without code changes.
        self.declare_parameter("leaf_size", 0.05)
        self.declare_parameter("max_height", 0.5)
        self.declare_parameter("max_depth", 3.0)
        self.declare_parameter("ground_ransac_threshold", 0.05)
        self.declare_parameter("wall_distance_threshold", 0.065)
        self.declare_parameter("wall_normal_threshold", 0.3)
        self.declare_parameter("cluster_tolerance", 0.1)
        self.declare_parameter("min_cluster_size", 15)
        self.declare_parameter("max_cluster_size", 95)

        self.leaf_size = self.get_parameter("leaf_size").value
        self.max_height = self.get_parameter("max_height").value
        self.max_depth = self.get_parameter("max_depth").value
        self.ground_ransac_threshold = self.get_parameter("ground_ransac_threshold").value
        self.wall_distance_threshold = self.get_parameter("wall_distance_threshold").value
        self.wall_normal_threshold = self.get_parameter("wall_normal_threshold").value
        self.cluster_tolerance = self.get_parameter("cluster_tolerance").value
        self.min_cluster_size = self.get_parameter("min_cluster_size").value
        self.max_cluster_size = self.get_parameter("max_cluster_size").value

        # Localization: storage for latest transformation data.
        self.latest_matrix = None
        self.latest_quaternion = None

        # Dynamic Transform Broadcaster (for publishing realtime TF transforms).
        self.br = TransformBroadcaster(self)

        # Subscribers for localization.
        self.create_subscription(Float64MultiArray, "/localization_matrix", self.matrix_callback, 10)
        self.create_subscription(Quaternion, "/localization_quaternion", self.quaternion_callback, 10)

        # Publishers for each processing stage.
        self.downsampled_pub = self.create_publisher(PointCloud2, '/downsampled_points', 10)
        self.transformed_pub = self.create_publisher(PointCloud2, '/transformed_points', 10)
        self.filtered_pub = self.create_publisher(PointCloud2, '/filtered_points', 10)  # Intermediate stage.
        self.ground_pub = self.create_publisher(PointCloud2, '/ground_points', 10)
        self.non_ground_pub = self.create_publisher(PointCloud2, '/non_ground_points', 10)
        self.wall_pub = self.create_publisher(PointCloud2, '/wall_points', 10)

        # Subscribe to raw point cloud from the camera.
        self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',  # Adjust if necessary.
            self.pointcloud_callback,
            10
        )

        # TF Buffer and Listener for lookup.
        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Logging rate limiter.
        self.last_log_time = time.time()
        self.log_interval = 3.0  # seconds

        self.get_logger().info('PointCloud Filtering Node started.')

    def should_log(self):
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            self.last_log_time = current_time
            return True
        return False

    # Callback for localization matrix.
    def matrix_callback(self, msg):
        if len(msg.data) == 16:
            self.latest_matrix = np.array(msg.data, dtype=np.float64).reshape((4, 4))
            self.get_logger().info("Localization matrix updated.")
            self.publish_transform()  # Publish updated TF transform.

    # Callback for localization quaternion.
    def quaternion_callback(self, msg):
        self.latest_quaternion = np.array([msg.w, msg.x, msg.y, msg.z], dtype=np.float64)
        self.get_logger().info("Localization quaternion updated.")

    def publish_transform(self):
        if self.latest_matrix is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"          # Global frame.
        t.child_frame_id = "base_link"       # Rover's frame.
        t.transform.translation.x = float(self.latest_matrix[0, 3])
        t.transform.translation.y = float(self.latest_matrix[1, 3])
        t.transform.translation.z = float(self.latest_matrix[2, 3])
        quat = mat2quat(self.latest_matrix[:3, :3])
        t.transform.rotation.x = quat[1]
        t.transform.rotation.y = quat[2]
        t.transform.rotation.z = quat[3]
        t.transform.rotation.w = quat[0]
        self.br.sendTransform(t)
        self.get_logger().info("Published dynamic transform from map to base_link.")

    def pointcloud_callback(self, msg):
        # Convert raw point cloud to NumPy array.
        raw_points = self.read_pointcloud(msg)
        if self.should_log():
            self.get_logger().info(f"Raw points count: {len(raw_points)}")
        
        # Downsample the point cloud.
        downsampled_points = self.downsample_pointcloud(raw_points, leaf_size=self.leaf_size)
        if self.should_log():
            self.get_logger().info(f"Downsampled points count: {len(downsampled_points)}")
        
        # Transform point cloud into base_link frame.
        transformed_points = self.transform_pointcloud(downsampled_points, msg.header.frame_id)
        if self.should_log():
            self.get_logger().info(f"Transformed points count: {len(transformed_points)}")
        
        # Apply height and depth filtering.
        filtered_points = self.filter_high_points(transformed_points, max_height=self.max_height)
        filtered_points = self.filter_far_points(filtered_points, max_depth=self.max_depth)
        if self.should_log():
            self.get_logger().info(f"Filtered points count: {len(filtered_points)}")
        self.publish_pointcloud(filtered_points, self.filtered_pub, "base_link")  # For debugging.
        
        # Segment ground using RANSAC.
        ground_points, non_ground_points = self.segment_ground(filtered_points)
        if self.should_log():
            self.get_logger().info(f"Ground points count: {len(ground_points)}, Non-ground points count: {len(non_ground_points)}")
        
        # Segment walls from non-ground points.
        wall_points, non_wall_points = self.segment_walls(non_ground_points, normal_threshold=self.wall_normal_threshold, distance_threshold=self.wall_distance_threshold)
        if self.should_log():
            self.get_logger().info(f"Wall points count: {len(wall_points)}, Remaining non-wall points count: {len(non_wall_points)}")
        
        # Apply clustering on remaining non-wall (obstacle) points.
        clustered_obstacle_points = self.apply_clustering(non_wall_points)
        if self.should_log():
            self.get_logger().info(f"Clustered obstacle points count: {len(clustered_obstacle_points)}")
        
        # Publish each stage.
        self.publish_pointcloud(downsampled_points, self.downsampled_pub, "base_link")
        self.publish_pointcloud(transformed_points, self.transformed_pub, "base_link")
        self.publish_pointcloud(ground_points, self.ground_pub, "base_link")
        self.publish_pointcloud(clustered_obstacle_points, self.non_ground_pub, "base_link")
        self.publish_pointcloud(wall_points, self.wall_pub, "base_link")

    def read_pointcloud(self, msg):
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        return np.array([(p[0], p[1], p[2]) for p in points], dtype=np.float32)

    def downsample_pointcloud(self, points, leaf_size=0.1):
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
        if self.latest_matrix is None:
            if self.should_log():
                self.get_logger().warn("No localization matrix received yet.")
            return points
        try:
            transform_stamped = self.tf_buffer.lookup_transform("base_link", source_frame, rclpy.time.Time())
            translation = np.array([
                transform_stamped.transform.translation.x,
                transform_stamped.transform.translation.y,
                transform_stamped.transform.translation.z
            ])
            quaternion = [
                transform_stamped.transform.rotation.w,
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z
            ]
            rotation_matrix = quat2mat(quaternion)
        except Exception as e:
            if self.should_log():
                self.get_logger().error(f"Transform lookup failed: {e}")
            return np.empty((0, 3), dtype=np.float32)
        return (points @ rotation_matrix.T) + translation

    def filter_high_points(self, points, max_height):
        filtered = points[points[:, 2] < max_height]
        if self.should_log():
            self.get_logger().info(f"Filter high: {len(filtered)} points remain under height {max_height}.")
        return filtered

    def filter_far_points(self, points, max_depth):
        filtered = points[points[:, 0] < max_depth]
        if self.should_log():
            self.get_logger().info(f"Filter far: {len(filtered)} points remain within depth {max_depth}m.")
        return filtered

    def segment_ground(self, points):
        if len(points) == 0:
            return np.empty((0, 3)), np.empty((0, 3))
        cloud = pcl.PointCloud(points.astype(np.float32))
        seg = cloud.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(self.ground_ransac_threshold)
        indices, coefficients = seg.segment()
        ground_points = points[indices] if indices else np.empty((0, 3))
        non_ground_points = np.delete(points, indices, axis=0) if indices else points
        if self.should_log():
            self.get_logger().info(f"Segmented ground: {len(ground_points)} ground points, {len(non_ground_points)} non-ground points. Coefficients: {coefficients}")
        return ground_points, non_ground_points

    def segment_walls(self, points, normal_threshold=0.6, distance_threshold=0.3):
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
                self.get_logger().info(f"Detected {len(wall_points)} wall points with coefficient abs({abs(coefficients[2]):.3f}).")
            return wall_points, remaining_points
        else:
            return np.empty((0, 3)), points

    def apply_clustering(self, points):
        if len(points) == 0:
            return points
        cloud = pcl.PointCloud(points.astype(np.float32))
        tree = cloud.make_kdtree()
        ec = cloud.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(self.cluster_tolerance)
        ec.set_MinClusterSize(self.min_cluster_size)
        ec.set_MaxClusterSize(self.max_cluster_size)
        ec.set_SearchMethod(tree)
        cluster_indices = ec.Extract()
        filtered_points = []
        for indices in cluster_indices:
            if len(indices) >= self.min_cluster_size and len(indices) <= self.max_cluster_size:
                for idx in indices:
                    filtered_points.append(points[idx])
        return np.array(filtered_points, dtype=np.float32)

    def publish_pointcloud(self, points, publisher, frame_id):
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
