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

        # Declare parameters for grid creation and smoothing.
        self.declare_parameter("grid_size", 100)  # Grid is grid_size x grid_size (e.g. 100x100)
        self.declare_parameter("resolution", 0.1)   # Size of each cell (meters per cell)
        self.declare_parameter("map_origin_x", -2.5)  # X-origin of the map in meters
        self.declare_parameter("map_origin_y", -2.5)  # Y-origin of the map in meters
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("update_rate", 2.0)  # Seconds between publishing
        self.declare_parameter("alpha", 0.5)  # EMA smoothing factor (0: no update, 1: no smoothing)
        self.declare_parameter("occupancy_threshold", 50)  # Threshold to mark cell as occupied
        # Parameters for optional manual inflation:
        self.declare_parameter("do_manual_inflation", False)  # Set to True to use manual inflation
        self.declare_parameter("inflation_radius", 0.5)  # Inflation radius in meters

        # Retrieve parameters.
        self.grid_size = self.get_parameter("grid_size").value
        self.resolution = self.get_parameter("resolution").value
        self.map_origin = [
            self.get_parameter("map_origin_x").value,
            self.get_parameter("map_origin_y").value
        ]
        self.frame_id = self.get_parameter("frame_id").value
        self.update_rate = self.get_parameter("update_rate").value
        self.alpha = self.get_parameter("alpha").value
        self.occupancy_threshold = self.get_parameter("occupancy_threshold").value
        self.do_manual_inflation = self.get_parameter("do_manual_inflation").value
        self.inflation_radius = self.get_parameter("inflation_radius").value

        # Subscriber: use the non-ground points filtered by your filtering.py node.
        self.sub = self.create_subscription(
            PointCloud2,
            '/non_ground_points',
            self.pointcloud_callback,
            10
        )

        # Publisher for the occupancy grid.
        self.occ_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        # Timer to publish the occupancy grid at a controlled rate.
        self.timer = self.create_timer(self.update_rate, self.publish_occupancy_grid)

        # Storage for the latest occupancy grid and for temporal smoothing.
        self.latest_grid = None
        self.smoothed_grid = None  # Will hold the exponentially smoothed grid

        self.get_logger().info('Occupancy Grid Node started.')

    def pointcloud_callback(self, msg):
        """
        Convert non-ground point cloud data (using x, y) into an occupancy grid.
        Each cell in the grid is set to 100 if an obstacle is detected.
        """
        # Convert PointCloud2 to a NumPy array (only x,y needed)
        points = np.array([
            (p[0], p[1])
            for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)
        ], dtype=np.float32)

        # Create an empty grid initialized to -1 (unknown)
        grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)

        # Convert point coordinates to grid indices and mark cell as occupied (100)
        for x, y in points:
            grid_x = int((x - self.map_origin[0]) / self.resolution)
            grid_y = int((y - self.map_origin[1]) / self.resolution)
            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                grid[grid_y, grid_x] = 100

        # Convert to binary grid: occupied cells as 100; else free (0)
        binary_grid = np.where(grid == 100, 100, 0).astype(np.int8)

        # Temporal smoothing using exponential moving average.
        if self.smoothed_grid is None:
            self.smoothed_grid = binary_grid.astype(np.float32)
        else:
            self.smoothed_grid = (self.alpha * binary_grid +
                                  (1 - self.alpha) * self.smoothed_grid)

        # Threshold the smoothed grid to generate the final occupancy grid.
        self.latest_grid = np.where(self.smoothed_grid > self.occupancy_threshold, 100, 0).astype(np.int8)

    def inflate_obstacles(self, grid):
        """
        Perform manual inflation (dilation) on the occupancy grid.
        Cells within the inflation radius from an occupied cell are marked as occupied.
        """
        inflation_cells = int(np.ceil(self.inflation_radius / self.resolution))
        inflated_grid = grid.copy()
        rows, cols = grid.shape
        # Loop over all cells.
        for i in range(rows):
            for j in range(cols):
                if grid[i, j] == 100:
                    # Mark neighbor cells within the inflation radius.
                    for di in range(-inflation_cells, inflation_cells + 1):
                        for dj in range(-inflation_cells, inflation_cells + 1):
                            if di**2 + dj**2 <= inflation_cells**2:
                                new_i = i + di
                                new_j = j + dj
                                if 0 <= new_i < rows and 0 <= new_j < cols:
                                    inflated_grid[new_i, new_j] = 100
        return inflated_grid

    def publish_occupancy_grid(self):
        """
        Publish the occupancy grid as a nav_msgs/OccupancyGrid.
        Optionally, apply manual inflation.
        """
        if self.latest_grid is None:
            return  # No data yet to publish

        grid_to_publish = self.latest_grid.copy()

        # Optionally, apply manual inflation if enabled.
        if self.do_manual_inflation:
            grid_to_publish = self.inflate_obstacles(grid_to_publish)

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

        # Flatten grid to a list.
        grid_msg.data = grid_to_publish.flatten().tolist()

        self.occ_pub.publish(grid_msg)
        self.get_logger().info("Published Occupancy Grid")

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
