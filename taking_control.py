import math
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from enum import Enum
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.action import Dock, Undock
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
import threading
import termios
import tty

# === CONFIGURABLE PARAMETERS ===
speed = 50  # Initial speed (0–100)
manual_mode = False
last_key_time = time.time()  # Used to track last manual input time
manual_control_keys = {'w': 'forward', 'a': 'left', 's': 'backward', 'd': 'right'}

# === NON-BLOCKING KEY INPUT FUNCTION ===
def get_key():
    """Get a keypress without blocking"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

# === ROBOT MOVEMENT PLACEHOLDER FUNCTIONS (REPLACED BY ROS2 VELOCITY PUBLISHING) ===
# def send_command(direction, speed):
#     """Simulate sending a command to the robot's motors"""
#     print(f"Moving {direction} at speed {speed}")

# def stop_robot():
#     """Simulate stopping the robot"""
#     print("Stopping robot")

# === AUTO MOVE FUNCTION (Feature 1 & 4) ===
def auto_move_thread(node):
    """Move robot automatically every 2 seconds in a loop, revert to auto mode after 5 seconds of no manual input"""
    global manual_mode, speed, last_key_time
    directions = ['forward', 'right', 'backward', 'left']
    i = 0
    while rclpy.ok():
        # Revert to auto mode after 5 seconds of no keypress (Feature 4)
        if not manual_mode or (time.time() - last_key_time) > 5:
            manual_mode = False
            direction = directions[i % 4]
            twist = Twist()
            linear_speed = speed / 100.0 * 0.2  # Scale speed to a reasonable linear velocity
            angular_speed = speed / 100.0 * 1.0 # Scale speed to a reasonable angular velocity
            if direction == 'forward':
                twist.linear.x = linear_speed
            elif direction == 'backward':
                twist.linear.x = -linear_speed * 0.5 # Backwards slower
            elif direction == 'left':
                twist.angular.z = angular_speed
            elif direction == 'right':
                twist.angular.z = -angular_speed
            node.out_pub_vel.publish(twist)
            print(f"Auto moving {direction} at speed {speed}")
            i += 1
            time.sleep(2)
        else:
            time.sleep(0.1)
    twist = Twist()
    node.out_pub_vel.publish(twist) # Stop on shutdown

# === MANUAL CONTROL FUNCTION (Feature 2) ===
def manual_move_command(node, key):
    """Move robot manually based on keypress"""
    global manual_mode, last_key_time, speed
    manual_mode = True
    last_key_time = time.time()  # Reset the manual control timer
    direction = manual_control_keys[key]
    twist = Twist()
    linear_speed = speed / 100.0 * 0.2  # Scale speed
    angular_speed = speed / 100.0 * 1.0 # Scale speed
    if direction == 'forward':
        twist.linear.x = linear_speed
    elif direction == 'backward':
        twist.linear.x = -linear_speed * 0.5
    elif direction == 'left':
        twist.angular.z = angular_speed
    elif direction == 'right':
        twist.angular.z = -angular_speed
    node.out_pub_vel.publish(twist)
    print(f"Manual moving {direction} at speed {speed}")

# === SPEED CONTROL FUNCTION (Feature 3) ===
def adjust_speed_command(increase=True):
    """Increase or decrease robot speed"""
    global speed
    if increase:
        speed = min(100, speed + 10)
        print(f"Increasing speed to {speed}")
    else:
        speed = max(0, speed - 10)
        print(f"Decreasing speed to {speed}")

class TurtleState(Enum):
    FORWARD = 1
    BACKWARD = 2
    TURN_LEFT = 3
    TURN_RIGHT = 4

class SafeNavigation(Node):
    def __init__(self, robot_name):
        super().__init__('safe_navigation_node')
        self.robot_name = robot_name

        self.cycle_dt = 0.05
        self.cycle_current = 0
        self.cycle_last_transition = 0
        self.time_since_last_transition = 0

        qos_profile = QoSProfile(depth=10)

        #subscription to read hazards
        self.in_sub_hazard = self.create_subscription(
            HazardDetectionVector,
            f'/{robot_name}/hazard_detection',
            self.read_hazard,
            qos_profile_sensor_data
        )
        #subscription to control velocity
        self.out_pub_vel = self.create_publisher(
            Twist,
            f'/{robot_name}/cmd_vel',
            qos_profile
        )
        #subscription to control position in the room
        self.odom_subscription = self.create_subscription(
            Odometry,
            f'/{robot_name}/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.current_position = (0.0, 0.0)
        self.current_yaw = 0.0

        self.dock_position = None

        self.undock_client = ActionClient(self, Undock, f'/{self.robot_name}/undock')
        self.dock_client = ActionClient(self, Dock, f'/{self.robot_name}/dock')

        self.state = TurtleState.FORWARD
        self.hazard_detected = False
        self.hazard_direction = None

        self.timer = self.create_timer(self.cycle_dt, self.control_cycle)

    def undock(self):
        self.get_logger().info('Starting undock...')

        # Checks if the undocking server is available
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Undocking server is not available')
            return

        goal_msg = Undock.Goal()
        send_goal_future = self.undock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Undocking request rejected')
            return

        self.get_logger().info('⏳ Undocking in progress...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info('✅ Undocking completed successfully!')

    def euler_from_quaternion(self, quat):
        x, y, z, w = quat
        # roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


    def navigate_to_dock(self, dock_position):
        dock_x, dock_y = dock_position

        # Calculate angle to dock
        dx = dock_x - self.current_position[0]
        dy = dock_y - self.current_position[1]
        desired_angle = math.atan2(dy, dx)

        # Compute angular difference
        angle_diff = desired_angle - self.current_yaw
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

        # 1. Rotate toward dock
        while abs(angle_diff) > 0.05:  # threshold: 0.05 rad ~ 3 degrees
            twist = Twist()
            twist.angular.z = 0.3 if angle_diff > 0 else -0.3
            self.out_pub_vel.publish(twist)

            # Recalculate angle_diff
            dx = dock_x - self.current_position[0]
            dy = dock_y - self.current_position[1]
            desired_angle = math.atan2(dy, dx)
            angle_diff = desired_angle - self.current_yaw
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

            rclpy.spin_once(self)

        # Stop rotating
        self.out_pub_vel.publish(Twist())

        # 2. Move forward until close to dock
        while math.hypot(dx, dy) > 0.2:  # distance threshold
            twist = Twist()
            twist.linear.x = 0.2
            self.out_pub_vel.publish(twist)

            dx = dock_x - self.current_position[0]
            dy = dock_y - self.current_position[1]

            rclpy.spin_once(self)

        # Stop
        self.out_pub_vel.publish(Twist())
        self.get_logger().info("Docking complete!")

    def dock(self):
        self.get_logger().info('🔋 Starting docking...')

        if not self.dock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Docking server is not available')
            return

        goal_msg = Dock.Goal()
        send_goal_future = self.dock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Docking request rejected')
            return

        self.get_logger().info('⏳ Docking in progress...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info('✅ Docking completed successfully!')

    def read_hazard(self, msg):
        self.hazard_detected = False
        for detection in msg.detections:
            if detection.type == 1:
                self.hazard_detected = True
                frame = detection.header.frame_id
                if frame in ["bump_left", "bump_front_left"]:
                    self.hazard_direction = "left"
                elif frame in ["bump_right", "bump_front_right"]:
                    self.hazard_direction = "right"
                elif frame in ["bump_center", "bump_front"]:
                    self.hazard_direction = "front"

    def odom_callback(self, msg):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )


        # Extract orientation in yaw
        orientation_q = msg.pose.pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        (_, _, yaw) = self.euler_from_quaternion(quaternion)
        self.current_yaw = yaw

    def output_forward(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        self.out_pub_vel.publish(msg)

    def output_backward(self):
        msg = Twist()
        msg.linear.x = -0.1
        msg.angular.z = 0.0
        self.out_pub_vel.publish(msg)

    def output_turn_left(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 1.0
        self.out_pub_vel.publish(msg)

    def output_turn_right(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -1.0
        self.out_pub_vel.publish(msg)

    def next_state(self):
        transition = False
        if self.state == TurtleState.FORWARD and self.hazard_detected:
            self.state = TurtleState.BACKWARD
            transition = True
        elif self.state == TurtleState.BACKWARD and self.evt_enough_time_spent(0.5):
            if self.hazard_direction == "left":
                self.state = TurtleState.TURN_RIGHT
            elif self.hazard_direction == "right":
                self.state = TurtleState.TURN_LEFT
            else:
                self.state = TurtleState.TURN_LEFT
            transition = True
        elif self.state in [TurtleState.TURN_LEFT, TurtleState.TURN_RIGHT] and self.evt_enough_time_spent(1.5):
            self.state = TurtleState.FORWARD
            transition = True

        self.cycle_current += 1
        if transition:
            self.cycle_last_transition = self.cycle_current
            self.time_since_last_transition = 0
        else:
            self.time_since_last_transition += self.cycle_dt

    def evt_enough_time_spent(self, duration):
        return self.time_since_last_transition > duration

def main(args=None):
    global manual_mode, last_key_time

    rclpy.init(args=args)

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        robot_name = input("Enter the robot name: ")
    print(f"Initializing SafeNavigationDock for {robot_name}...")

    node = SafeNavigation(robot_name)

    print("Control the robot with:")
    print("  w/a/s/d = manual control")
    print("  +/-     = increase/decrease speed")
    print("  q       = quit")
    print("Auto movement starts by default...")

    # Start auto-move thread (Feature 1)
    auto_thread = threading.Thread(target=auto_move_thread, args=(node,))
    auto_thread.daemon = True
    auto_thread.start()

    try:
        while rclpy.ok():
            key = get_key()

            # Feature 5: Quit
            if key == 'q':
                print("Quitting...")
                break

            # Feature 2: Manual Control
            elif key in manual_control_keys:
                manual_move_command(node, key)

            # Feature 3: Speed Adjustment
            elif key == '+':
                adjust_speed_command(increase=True)
            elif key == '-':
                adjust_speed_command(increase=False)

            # Reset manual mode timeout timer
            last_key_time = time.time()

            rclpy.spin_once(node, timeout_sec=0.1) # Keep the ROS 2 node alive

        # Stop the robot before exiting
        twist = Twist()
        node.out_pub_vel.publish(twist)

    except KeyboardInterrupt:
        print("\nInterrupted by user. Stopping robot.")
        twist = Twist()
        node.out_pub_vel.publish(twist)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()