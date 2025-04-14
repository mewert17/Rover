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
        self.counter_grid = None  # Grid holding the observation count for each cell

        # Counter method parameters
        self.counter_threshold = 3  # Number of consecutive observations required to mark as occupied
        self.decay_value = 1        # How much to decrement the counter when a cell is not observed

        # Grid parameters
        self.grid_size = 100            # Dimensions: grid_size x grid_size cells
        self.resolution = 0.1           # Grid cell size (meters per cell)
        self.map_origin = [-2.5, -2.5]    # Map origin (X, Y) in meters
        self.frame_id = "camera_link"   # Frame in which the occupancy grid is published

        self.get_logger().info('Counter-Based Occupancy Grid Node started.')

    def pointcloud_callback(self, msg):
        """
        Process the incoming PointCloud2 message to update the occupancy grid.
        Uses a counter/accumulation method that only marks a cell as occupied if it is consistently observed.
        """
        # Convert PointCloud2 to a NumPy array extracting only the x and y coordinates.
        points = np.array([
            (p[0], p[1]) for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)
        ], dtype=np.float32)

        # Create an empty binary grid (initialize with 0 for free cells;
        # we use -1 in intermediate raw grid, but then convert to binary below)
        raw_grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)
        
        # Convert each (x, y) to grid indices and mark as occupied (100).
        for x, y in points:
            grid_x = int((x - self.map_origin[0]) / self.resolution)
            grid_y = int((y - self.map_origin[1]) / self.resolution)
            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                raw_grid[grid_y, grid_x] = 100  # Mark cell as occupied

        # Convert the raw grid into a simple binary grid: 100 for occupied, else 0.
        binary_grid = np.where(raw_grid == 100, 100, 0).astype(np.int8)

        # Initialize the counter grid if needed.
        if self.counter_grid is None:
            self.counter_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int32)

        # Update the counter grid:
        # - If a cell is occupied in the current frame, increment its counter.
        # - Otherwise, decrement the counter (clamped at 0) to allow false positives to fade.
        # We use vectorized operations to update the counter grid.
        increment_mask = (binary_grid == 100)
        decrement_mask = (binary_grid == 0)

        self.counter_grid[increment_mask] += 1
        # Ensure cells do not go below zero.
        self.counter_grid[decrement_mask] = np.maximum(self.counter_grid[decrement_mask] - self.decay_value, 0)

    def publish_occupancy_grid(self):
        """
        Publish the persistent occupancy grid as a nav_msgs/OccupancyGrid message.
        A cell is marked as occupied (100) only if its counter exceeds the defined threshold.
        """
        if self.counter_grid is None:
            return  # No data received yet

        # Generate the persistent (final) occupancy grid.
        persistent_grid = np.where(self.counter_grid >= self.counter_threshold, 100, 0).astype(np.int8)

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

        # Flatten the persistent grid into a list, as required by the OccupancyGrid message.
        grid_msg.data = persistent_grid.flatten().tolist()

        self.occ_pub.publish(grid_msg)
        self.get_logger().info("Published Counter-Based Occupancy Grid")

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
