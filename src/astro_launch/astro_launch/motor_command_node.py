#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class MotorCommandNode(Node):
    def __init__(self):
        super().__init__('motor_command_node')

        # Subscribe to Nav2's output velocity commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publish motor command array: [left_motor_power, right_motor_power]
        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            '/command_velocity',
            10
        )

        self.get_logger().info("Motor Command Node Initialized")

        # Static slip factor (to be replaced later with dynamic estimation)
        self.slip_factor = 0.95  # Tweak depending on test results

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback function that processes /cmd_vel input
        and outputs left/right motor power in range [-1, 1].
        """
        linear_vel = msg.linear.x     # m/s forward
        angular_vel = msg.angular.z   # rad/s rotation

        # --- Rover-specific tuning ---
        # Assume tank steering (skid steer): angular controls differential speeds
        rover_width = 1.0  # meters — actual width of your rover (1.0m here)

        # Calculate raw left/right wheel speeds
        left_speed = linear_vel - (angular_vel * rover_width / 2.0)
        right_speed = linear_vel + (angular_vel * rover_width / 2.0)

        # Apply slip factor (optional — can be dynamic later)
        left_speed *= self.slip_factor
        right_speed *= self.slip_factor

        # Normalize both to range [-1, 1]
        max_speed = max(abs(left_speed), abs(right_speed), 1.0)  # Prevent divide by 0
        left_motor_power = left_speed / max_speed
        right_motor_power = right_speed / max_speed

        # Publish as Float32MultiArray
        motor_msg = Float32MultiArray()
        # Assume motors 0–2 = Left, 3–5 = Right
        motor_msg.data = [
            left_motor_power, left_motor_power, left_motor_power,
            right_motor_power, right_motor_power, right_motor_power
        ]

        self.motor_pub.publish(motor_msg)

        self.get_logger().info(f"Published motor powers: Left={left_motor_power:.2f}, Right={right_motor_power:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down motor_command_node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
