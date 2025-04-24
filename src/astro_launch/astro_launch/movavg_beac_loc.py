#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Quaternion, PoseStamped
import serial
import time
import datetime
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from collections import deque


class RoverLocalization(Node):
	def __init__(self):
		super().__init__('rover_localization')

		self.tf_broadcaster = TransformBroadcaster(self)

		# Serial port parameters.
		self.baudrate = 115200
		self.port1 = "/dev/ttyACM0"
		self.port2 = "/dev/ttyACM1"

		# Initialize serial connections.
		self.serial_conn_1 = self.initialize_dwm(self.port1)
		self.serial_conn_2 = self.initialize_dwm(self.port2)

		# Store latest tag positions by ID.
		self.tag_positions = {}

		# Publishers.
		self.matrix_publisher = self.create_publisher(Float64MultiArray, '/localization_matrix', 10)
		self.quaternion_publisher = self.create_publisher(Quaternion, '/localization_quaternion', 10)
		self.pose_publisher = self.create_publisher(PoseStamped, '/rover_pose', 10)

		# Timer at 10 Hz.
		self.timer = self.create_timer(0.1, self.read_and_publish_pose)

		# Moving average window length.
		self.ma_window = 5  # ← adjust this single variable to change smoothing length
		# Buffers for last N samples.
		self.mid_x_hist   = deque(maxlen=self.ma_window)
		self.mid_y_hist   = deque(maxlen=self.ma_window)
		self.mid_z_hist   = deque(maxlen=self.ma_window)
		self.heading_hist = deque(maxlen=self.ma_window)

	def initialize_dwm(self, port):
		"""Initialize a DWM serial connection on the given port."""
		try:
			dwm = serial.Serial(port=port, baudrate=self.baudrate, timeout=1)
			self.get_logger().info(f"Connected to {dwm.name}")
			dwm.write("\r\r".encode())
			time.sleep(1)
			return dwm
		except serial.SerialException as e:
			self.get_logger().error(f"Failed to open {port}: {e}")
			return None

	def parse_position_data(self, line):
		"""Parse a line like 'B0 ... POS:[x,y,z,...]' → (tag_id, x, y, z)."""
		try:
			decoded = line.decode().strip()
			tokens = decoded.split()
			if len(tokens) < 3:
				return None
			tag_id = tokens[0]
			pos_token = next((t for t in tokens if t.startswith("POS:[")), None)
			if not pos_token:
				return None
			inside = pos_token[pos_token.find('[')+1 : pos_token.find(']')]
			nums = inside.split(',')
			if len(nums) < 3:
				return None
			x, y, z = float(nums[0]), float(nums[1]), float(nums[2])
			return tag_id, x, y, z
		except Exception as e:
			self.get_logger().warn(f"Error parsing line '{line}': {e}")
			return None

	def read_serial(self, conn):
		"""Read and parse one line from serial."""
		if not conn:
			return None
		try:
			line = conn.readline()
			if line:
				return self.parse_position_data(line)
		except Exception as e:
			self.get_logger().error(f"Serial read error: {e}")
		return None

	def read_and_publish_pose(self):
		# Read both tags.
		for conn in (self.serial_conn_1, self.serial_conn_2):
			res = self.read_serial(conn)
			if res:
				tag_id, x, y, z = res
				self.tag_positions[tag_id] = (x, y, z)
				self.get_logger().debug(f"Tag {tag_id}: x={x}, y={y}, z={z}")

    	# Proceed only if both B0 and B1 are present.
		if "B0" in self.tag_positions and "B1" in self.tag_positions:
			back = self.tag_positions["B0"]
			front = self.tag_positions["B1"]
			# Guard against NaNs.
			if any(math.isnan(v) for v in (*back, *front)):
				self.get_logger().info("Skipping NaN readings.")
				return

			# Raw midpoint and heading.
			mid_x = (back[0] + front[0]) / 2.0
			mid_y = (back[1] + front[1]) / 2.0
			mid_z = (back[2] + front[2]) / 2.0
			dx, dy = front[0] - back[0], front[1] - back[1]
			orig_heading = math.atan2(dy, dx)
			adjusted_heading = orig_heading - (math.pi / 4)  # rotate 45° right

			# —— moving average ——
			self.mid_x_hist.append(mid_x)
			self.mid_y_hist.append(mid_y)
			self.mid_z_hist.append(mid_z)
			self.heading_hist.append(adjusted_heading)

			avg_mid_x = sum(self.mid_x_hist) / len(self.mid_x_hist)
			avg_mid_y = sum(self.mid_y_hist) / len(self.mid_y_hist)
			avg_mid_z = sum(self.mid_z_hist) / len(self.mid_z_hist)
			avg_heading = sum(self.heading_hist) / len(self.heading_hist)

			# Publish matrix
			c, s = math.cos(avg_heading), math.sin(avg_heading)
			mat = [
				c, -s, 0, avg_mid_x,
				s,  c, 0, avg_mid_y,
				0,  0, 1, avg_mid_z,
				0,  0, 0, 1
			]
			mat_msg = Float64MultiArray(data=[float(v) for v in mat])
			self.matrix_publisher.publish(mat_msg)

			# Publish quaternion
			qz = math.sin(avg_heading/2.0)
			qw = math.cos(avg_heading/2.0)
			quat = Quaternion(x=0.0, y=0.0, z=float(qz), w=float(qw))
			self.quaternion_publisher.publish(quat)

			# Publish PoseStamped
			pose = PoseStamped()
			pose.header.stamp = self.get_clock().now().to_msg()
			pose.header.frame_id = "map"
			pose.pose.position.x = avg_mid_x
			pose.pose.position.y = avg_mid_y
			pose.pose.position.z = avg_mid_z
			pose.pose.orientation = quat
			self.pose_publisher.publish(pose)

			# Broadcast TF
			tf = TransformStamped()
			tf.header.stamp = self.get_clock().now().to_msg()
			tf.header.frame_id = "odom"
			tf.child_frame_id = "base_link"
			tf.transform.translation.x = avg_mid_x
			tf.transform.translation.y = avg_mid_y
			tf.transform.translation.z = 0.0
			tf.transform.rotation = quat
			self.tf_broadcaster.sendTransform(tf)

		else:
			self.get_logger().debug("Waiting for both B0 and B1.")

	def shutdown(self):
    	# Close serial ports cleanly.
		for conn, port in ((self.serial_conn_1, self.port1), (self.serial_conn_2, self.port2)):
			if conn:
				conn.write("\r".encode())
				conn.close()
				self.get_logger().info(f"Closed {port}")

def main(args=None):
	rclpy.init(args=args)
	node = RoverLocalization()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		node.get_logger().info("Interrupted, shutting down...")
	finally:
		node.shutdown()
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()


