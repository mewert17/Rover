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

        # Subscriber to non-ground points
        self.sub = self.create_subscription(
            PointCloud2,
            '/non_ground_points',
            self.pointcloud_callback,
            10
        )

        # Publisher for the occupancy grid
        self.occ_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        # Timer for rate limiting
        self.timer = self.create_timer(2.0, self.publish_occupancy_grid)  # Publish every 2.0 seconds

        # Storage for grid and smoothing state
        self.latest_grid = None
        self.smoothed_grid = None  # This will hold our temporal smoothed grid

        # Smoothing parameters
        self.alpha = 0.5  # Smoothing factor; adjust between 0 (no update) and 1 (no smoothing)
        self.occupancy_threshold = 50  # Threshold to decide if a cell is occupied

        # Grid parameters
        self.grid_size = 100  # Number of grid cells (100x100)
        self.resolution = 0.1  # Grid cell size in meters (0.1m per cell)
        self.map_origin = [-2.5, -2.5]  # Origin (X, Y) in meters
        self.frame_id = "camera_link"  # Frame ID

        self.get_logger().info('Occupancy Grid Node started.')

    def pointcloud_callback(self, msg):
        """Convert non-ground points into an occupancy grid and update the smoothed grid."""
        # Convert PointCloud2 to NumPy array (only using x,y)
        points = np.array([
            (p[0], p[1]) for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)
        ], dtype=np.float32)

        # Create an empty grid (initialize as -1 for unknown)
        grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)

        # Convert point coordinates to grid indices
        for x, y in points:
            grid_x = int((x - self.map_origin[0]) / self.resolution)
            grid_y = int((y - self.map_origin[1]) / self.resolution)

            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                grid[grid_y, grid_x] = 100  # Mark as occupied

        # For smoothing, we first convert grid to a binary grid: occupied=100, else free=0.
        binary_grid = np.where(grid == 100, 100, 0).astype(np.int8)

        # Update our smoothed grid:
        # If this is the first frame, initialize smoothed_grid with the current binary grid.
        if self.smoothed_grid is None:
            self.smoothed_grid = binary_grid.astype(np.float32)
        else:
            # Apply exponential moving average per cell.
            self.smoothed_grid = (self.alpha * binary_grid + (1 - self.alpha) * self.smoothed_grid)

        # Threshold the smoothed grid to create a final occupancy grid.
        # Cells above occupancy_threshold become 100 (occupied), below become 0 (free).
        self.latest_grid = np.where(self.smoothed_grid > self.occupancy_threshold, 100, 0).astype(np.int8)

    def publish_occupancy_grid(self):
        """Publish the latest (smoothed) occupancy grid at a controlled rate."""
        if self.latest_grid is None:
            return  # No grid to publish yet

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

        # Convert the latest grid (NumPy array) to a flat list for the message.
        grid_msg.data = self.latest_grid.flatten().tolist()

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
