# here we merged the undocking with the navigation and started trying to dock the robot.

import math
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from enum import Enum
from irobot_create_msgs.msg import HazardDetectionVector # type: ignore
from irobot_create_msgs.action import Dock, Undock # type: ignore
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry


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
        self.get_logger().info('Initiating undock...')

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
        attempt = 1

        while True:
            self.get_logger().info(f"➡️ Docking attempt #{attempt}...")
            attempt += 1

            dock_x, dock_y = dock_position

            # 1. Rotate towards the dock
            dx = dock_x - self.current_position[0]
            dy = dock_y - self.current_position[1]
            desired_angle = math.atan2(dy, dx)
            angle_diff = desired_angle - self.current_yaw
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

            while abs(angle_diff) > 0.05:
                if self.hazard_detected:
                    self.get_logger().warn("❌ Collision detected during rotation. Cancelling docking...")
                    self.return_to_normal_behavior()
                    break

                twist = Twist()
                twist.angular.z = 0.3 if angle_diff > 0 else -0.3
                self.out_pub_vel.publish(twist)

                rclpy.spin_once(self)
                dx = dock_x - self.current_position[0]
                dy = dock_y - self.current_position[1]
                desired_angle = math.atan2(dy, dx)
                angle_diff = desired_angle - self.current_yaw
                angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
            else:
                self.out_pub_vel.publish(Twist())  # Stop rotation

                # 2. Move straight towards the dock
                while math.hypot(dx, dy) > 0.2:
                    if self.hazard_detected:
                        self.get_logger().warn("❌ Collision detected during approach. Cancelling docking...")
                        self.return_to_normal_behavior()
                        break

                    twist = Twist()
                    twist.linear.x = 0.2
                    self.out_pub_vel.publish(twist)

                    rclpy.spin_once(self)
                    dx = dock_x - self.current_position[0]
                    dy = dock_y - self.current_position[1]
                else:
                    self.out_pub_vel.publish(Twist())
                    self.get_logger().info("✅ Approach completed! Initiating final docking.")
                    self.dock()
                    break  # Successful docking

    def return_to_normal_behavior(self):
        self.get_logger().info("⏪ Returning to random mode for a few seconds...")

        start_time = time.time()
        self.state = TurtleState.FORWARD
        self.cycle_last_transition = self.cycle_current
        self.time_since_last_transition = 0.0

        while time.time() - start_time < 5:  # "normal" time before attempting docking again
            rclpy.spin_once(self)


    def dock(self):
        self.get_logger().info('🔋 Initiating docking...')

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

    def control_cycle(self):
        if self.state == TurtleState.FORWARD:
            self.output_forward()
        elif self.state == TurtleState.BACKWARD:
            self.output_backward()
        elif self.state == TurtleState.TURN_LEFT:
            self.output_turn_left()
        elif self.state == TurtleState.TURN_RIGHT:
            self.output_turn_right()

        self.next_state()

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
    rclpy.init(args=args)

    robot_name = sys.argv[1] if len(sys.argv) > 1 else input("Enter the robot name: ")
    print(f"Initializing SafeNavigationDock for {robot_name}...")

    node = SafeNavigation(robot_name)

    try:
        # Step 1: undock + 180° turn
        node.undock()

        # Wait for odometry to be updated
        time.sleep(0.1)
        node.dock_position = node.current_position
        print(f"Dock station position saved: {node.dock_position}")


        # Step 2: navigate avoiding obstacles for 20 seconds
        print("Navigating for 20 seconds before docking...")
        start_time = time.time()
        while time.time() - start_time < 20:
            rclpy.spin_once(node)

        # Step 3: automatic docking
        print(f"Current robot position: {node.current_position}")
        node.navigate_to_dock(node.dock_position)
        node.dock()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()