#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_node')

        # Subscriber to non-ground points (dynamic obstacles)
        self.sub = self.create_subscription(
            PointCloud2,
            '/non_ground_points',
            self.pointcloud_callback,
            10
        )

        # Publisher for the occupancy grid
        self.occ_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        # Timer for rate limiting publication
        self.timer = self.create_timer(2.0, self.publish_occupancy_grid)  # Publish every 2 seconds

        # Storage for grid processing
        self.smoothed_grid = None   # Temporary grid computed with exponential moving average (EMA)
        self.persistent_grid = None # Persistent occupancy grid (binary fusion/union of obstacles)

        # Smoothing parameters (for EMA)
        self.alpha = 0.5                # Smoothing factor: 0 (no update) to 1 (no smoothing)
        self.occupancy_threshold = 50   # Threshold (after smoothing) above which a cell is considered occupied

        # Grid parameters
        self.grid_size = 100            # Grid dimensions: grid_size x grid_size cells
        self.resolution = 0.1           # Grid cell size in meters (0.1m per cell)
        self.map_origin = [-2.5, -2.5]    # Map origin (X, Y) in meters
        self.frame_id = "camera_link"   # Frame in which the occupancy grid is published

        self.get_logger().info('Robust Occupancy Grid Node started.')

    def pointcloud_callback(self, msg):
        """
        Process the incoming PointCloud2 message to update the occupancy grid.
        Uses EMA on the binary grid, then updates the persistent grid with binary fusion
        so that once a cell is marked as occupied, it remains occupied.
        """
        # Convert PointCloud2 into a NumPy array extracting only x and y coordinates.
        points = np.array([
            (p[0], p[1]) for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)
        ], dtype=np.float32)

        # Create an empty grid (initialize as -1 for unknown)
        grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)

        # Convert point coordinates into grid indices and mark those cells as occupied (100)
        for x, y in points:
            grid_x = int((x - self.map_origin[0]) / self.resolution)
            grid_y = int((y - self.map_origin[1]) / self.resolution)
            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                grid[grid_y, grid_x] = 100  # Mark as occupied

        # Convert the raw grid into a binary grid (occupied = 100, free/unknown = 0)
        binary_grid = np.where(grid == 100, 100, 0).astype(np.int8)

        # Update the EMA-based smoothed grid
        if self.smoothed_grid is None:
            self.smoothed_grid = binary_grid.astype(np.float32)
        else:
            self.smoothed_grid = (self.alpha * binary_grid + (1 - self.alpha) * self.smoothed_grid)

        # Threshold the smoothed grid: above occupancy_threshold becomes occupied (100), else free (0)
        current_grid = np.where(self.smoothed_grid > self.occupancy_threshold, 100, 0).astype(np.int8)

        # Binary Fusion (Union) to update the persistent grid:
        # Once a cell is marked occupied, it remains occupied.
        if self.persistent_grid is None:
            self.persistent_grid = current_grid.copy()
        else:
            self.persistent_grid = np.maximum(self.persistent_grid, current_grid)

    def publish_occupancy_grid(self):
        """Publish the persistent occupancy grid as a nav_msgs/OccupancyGrid message."""
        if self.persistent_grid is None:
            return  # Nothing to publish yet

        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = self.frame_id

        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.grid_size
        grid_msg.info.height = self.grid_size
        grid_msg.info.origin.position.x = self.map_origin[0]
        grid_msg.info.origin.position.y = self.map_origin[1]
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.x = 0.0
        grid_msg.info.origin.orientation.y = 0.0
        grid_msg.info.origin.orientation.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        # Flatten the persistent grid to a list (expected by the OccupancyGrid message)
        grid_msg.data = self.persistent_grid.flatten().tolist()

        self.occ_pub.publish(grid_msg)
        self.get_logger().info("Published Robust Occupancy Grid")

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
