import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Float32MultiArray
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
import math

# --- States ---
STATE_MANUAL = 'MANUAL'
STATE_WAITING = 'WAITING_ON_COMMAND'
STATE_NAV1 = 'NAVI_1'
STATE_EXCA = 'EXCA'
STATE_NAV2 = 'NAVI_2'
STATE_DEPO = 'DEPO'
STATE_DETECT_REG = 'DETECT_REG'

# --- OPC UA Status Codes ---
STATUS_NOT_STARTED = 0
STATUS_ACK = 10
STATUS_STARTED = 20
STATUS_FINISHED = 30
STATUS_ERROR = 99

# --- Pi Commands ---
CMD_NAVI = 10
CMD_EXCA = 20
CMD_DEPO = 30
CMD_ERROR = 99

class RoutineNode(Node):
    def __init__(self):
        super().__init__('routine_node')
        self.state = STATE_WAITING
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_goal_active = False
        self.nav_goal_done = False
        self.nav_success = False

        self.navigation_status = STATUS_NOT_STARTED
        self.excavation_status = STATUS_NOT_STARTED
        self.deposition_status = STATUS_NOT_STARTED
        self.automation_state = ''

        # Subscriptions
        self.create_subscription(Int32, 'navigation_status', self.navi_status_callback, 10)
        self.create_subscription(Int32, 'excavation_status', self.exca_status_callback, 10)
        self.create_subscription(Int32, 'deposition_status', self.depo_status_callback, 10)
        self.create_subscription(String, 'automation_state', self.auto_state_callback, 10)

        # Publishers
        self.pi_command_pub = self.create_publisher(Int32, 'pi_command', 10)
        self.complete_task_pub = self.create_publisher(Int32, 'complete_task', 10)

        self.timer = self.create_timer(1.0, self.state_machine_loop)

    def navi_status_callback(self, msg):
        self.navigation_status = msg.data

    def exca_status_callback(self, msg):
        self.excavation_status = msg.data

    def depo_status_callback(self, msg):
        self.deposition_status = msg.data

    def auto_state_callback(self, msg):
        self.automation_state = msg.data

    def state_machine_loop(self):
        if self.automation_state == "Manual":
            self.state = STATE_MANUAL

        if self.state == STATE_MANUAL:
            return

        if self.state == STATE_WAITING:
            self.send_complete_task(0)
            self.send_pi_command(CMD_NAVI)
            if self.navigation_status == STATUS_ACK:
                self.send_complete_task(1)
                self.state = STATE_NAV1

        elif self.state == STATE_NAV1:
            if not self.nav_goal_active and not self.nav_goal_done:
                self.send_navigation_goal(1.35, 2.2, 0.0)
            elif self.nav_goal_done:
                if self.nav_success:
                    self.send_complete_task(49)
                    self.state = STATE_EXCA
                else:
                    self.state = STATE_WAITING

        elif self.state == STATE_EXCA:
            self.send_pi_command(CMD_EXCA)
            if self.excavation_status == STATUS_ACK:
                self.send_complete_task(1)
            if self.excavation_status == STATUS_FINISHED:
                self.send_complete_task(0)
                self.send_pi_command(CMD_NAVI)
                self.state = STATE_NAV2

        elif self.state == STATE_NAV2:
            if not self.nav_goal_active and not self.nav_goal_done:
                self.send_navigation_goal(1.0, 2.0, 0.0)
            elif self.nav_goal_done:
                if self.nav_success:
                    self.send_complete_task(49)
                    self.state = STATE_DEPO
                else:
                    self.state = STATE_WAITING

        elif self.state == STATE_DEPO:
            self.send_pi_command(CMD_DEPO)
            if self.deposition_status == STATUS_ACK:
                self.send_complete_task(1)
            if self.deposition_status == STATUS_FINISHED:
                self.state = STATE_DETECT_REG

        elif self.state == STATE_DETECT_REG:
            self.get_logger().info("Detecting regolith... Done.")
            self.state = STATE_WAITING

    def send_pi_command(self, value):
        msg = Int32()
        msg.data = value
        self.pi_command_pub.publish(msg)
        self.get_logger().info(f"Sent PiCommand: {value}")

    def send_complete_task(self, value):
        msg = Int32()
        msg.data = value
        self.complete_task_pub.publish(msg)
        self.get_logger().info(f"Sent CompleteTask: {value}")

    def send_navigation_goal(self, x, y, theta):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        quat = Quaternion()
        quat.z = math.sin(theta / 2.0)
        quat.w = math.cos(theta / 2.0)
        goal.pose.pose.orientation = quat

        self.nav_goal_active = True
        self.nav_goal_done = False
        self.nav_success = False

        self._send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal, feedback_callback=self.nav_feedback_callback)
        self._send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal was rejected")
            self.nav_goal_done = True
            self.nav_success = False
            return

        self.get_logger().info("Navigation goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info("Navigation succeeded")
            self.nav_success = True
        else:
            self.get_logger().error(f"Navigation failed: status {status}")
            self.nav_success = False
        self.nav_goal_done = True
        self.nav_goal_active = False

    def nav_feedback_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = RoutineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
