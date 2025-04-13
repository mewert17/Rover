import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from transforms3d.quaternions import quat2mat
from std_msgs.msg import Header
import pcl  # Requires python-pcl
import time  # For delays

class PointCloudFilteringNode(Node):
    def __init__(self):
        super().__init__('pointcloud_filtering_node')

        # Publishers for various stages
        self.downsampled_pub = self.create_publisher(PointCloud2, '/downsampled_points', 10)
        self.transformed_pub = self.create_publisher(PointCloud2, '/transformed_points', 10)
        self.ground_pub = self.create_publisher(PointCloud2, '/ground_points', 10)
        self.non_ground_pub = self.create_publisher(PointCloud2, '/non_ground_points', 10)
        # Optional: wall points for debugging
        self.wall_pub = self.create_publisher(PointCloud2, '/wall_points', 10)

        # Subscriber to raw point cloud from camera
        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',  # Adjust as needed
            self.pointcloud_callback,
            10
        )

        # TF Buffer and Listener to transform the point cloud to the target frame
        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Rate limiting parameters for logging
        self.last_log_time = time.time()
        self.log_interval = 3.0  # seconds

        self.get_logger().info('PointCloud Filtering Node started.')

    def should_log(self):
        """Check if enough time has passed to log a message."""
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            self.last_log_time = current_time
            return True
        return False

    def pointcloud_callback(self, msg):
        """Process and filter point cloud data."""
        # Convert the raw PointCloud2 message into a NumPy array and downsample
        raw_points = self.read_pointcloud(msg)
        downsampled_points = self.downsample_pointcloud(raw_points, leaf_size=0.04)

        # Transform the downsampled point cloud to the "camera_link" frame
        transformed_points = self.transform_pointcloud(downsampled_points, msg.header.frame_id)

        # Pre-filter: remove points above a certain height and points too far away
        filtered_points = self.filter_high_points(transformed_points, max_height=0.2)
        filtered_points = self.filter_far_points(filtered_points, max_depth=3.0)

        # Segment ground from non-ground points using RANSAC
        ground_points, non_ground_points = self.segment_ground(filtered_points)

        # Further segment vertical wall planes from the non-ground points using RANSAC.
        wall_points, non_wall_points = self.segment_walls(non_ground_points, normal_threshold=0.3, distance_threshold=0.08)

        # Apply Euclidean clustering on non-wall points with relaxed cluster size filtering.
        # Clusters with at least 30 points and no more than 1000 points are retained.
        clustered_obstacle_points = self.apply_clustering(non_wall_points)

        # Publish all results. The final obstacle cloud (post-clustering) is published on non_ground_pub.
        self.publish_pointcloud(downsampled_points, self.downsampled_pub, "camera_link")
        self.publish_pointcloud(transformed_points, self.transformed_pub, "camera_link")
        self.publish_pointcloud(ground_points, self.ground_pub, "camera_link")
        self.publish_pointcloud(clustered_obstacle_points, self.non_ground_pub, "camera_link")
        # Publish wall points for debugging (optional)
        self.publish_pointcloud(wall_points, self.wall_pub, "camera_link")

    def read_pointcloud(self, msg):
        """Convert the raw PointCloud2 message into a NumPy array."""
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        return np.array([(p[0], p[1], p[2]) for p in points], dtype=np.float32)

    def downsample_pointcloud(self, points, leaf_size=0.03):
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
        """Transform the point cloud to the target frame ("camera_link")."""
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                "camera_link",  # Ensure this frame exists in your TF tree
                source_frame,
                rclpy.time.Time()
            )
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
        seg.set_distance_threshold(0.05)  # Tolerance for ground segmentation

        indices, coefficients = seg.segment()
        ground_points = points[indices] if indices else np.empty((0, 3))
        non_ground_points = np.delete(points, indices, axis=0) if indices else points

        if self.should_log():
            self.get_logger().info(f"Segmented ground: {len(ground_points)} points, {len(non_ground_points)} non-ground points.")
        return ground_points, non_ground_points

    def segment_walls(self, points, normal_threshold=0.6, distance_threshold=0.3):
        """
        Segment vertical wall planes from non-ground points using RANSAC.
        
        Args:
            points (np.array): N x 3 array of non-ground points.
            normal_threshold (float): Threshold for the z-component of the plane normal.
                                      If |c| (from plane coefficients [a,b,c,d]) is below this value,
                                      the plane is considered vertical.
            distance_threshold (float): RANSAC distance threshold.
        
        Returns:
            tuple: (wall_points, remaining_points)
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

        # For a vertical wall, the z component (|c|) should be small.
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
        Only clusters with at least 30 points and no more than 1000 points are retained.
        """
        if len(points) == 0:
            return points

        # Convert numpy array to a PCL point cloud.
        cloud = pcl.PointCloud(points.astype(np.float32))

        # Create a KD-Tree for the point cloud.
        tree = cloud.make_kdtree()

        # Setup Euclidean Clustering with relaxed thresholds.
        ec = cloud.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.1)  # Adjust based on your resolution.
        ec.set_MinClusterSize(15)     # Lower minimum cluster size.
        ec.set_MaxClusterSize(250)   # Increase maximum cluster size.
        ec.set_SearchMethod(tree)

        # Extract clusters (each cluster is a list of point indices).
        cluster_indices = ec.Extract()

        filtered_points = []
        for indices in cluster_indices:
            if len(indices) >= 15 and len(indices) <= 500:  # Valid obstacle cluster.
                for idx in indices:
                    filtered_points.append(points[idx])
        return np.array(filtered_points, dtype=np.float32)

    def publish_pointcloud(self, points, publisher, frame_id):
        """Publish a point cloud as a PointCloud2 messae."""
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
