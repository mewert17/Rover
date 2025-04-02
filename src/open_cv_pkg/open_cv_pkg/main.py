import rclpy
from object_detection import ObjectDetectionNode  # Assuming object_detection.py is in the same directory

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2
    node = ObjectDetectionNode()  # Create the object detection node
    
    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # Gracefully shut down the node
        rclpy.shutdown()  # Shutdown ROS2
