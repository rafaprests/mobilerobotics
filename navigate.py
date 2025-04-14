import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data

from rclpy.qos import QoSProfile
from enum import Enum

from irobot_create_msgs.msg import HazardDetectionVector

# Enumerating the states of the FSM
class TurtleState(Enum):
    FORWARD = 1
    BACKWARD = 2
    TURN_LEFT = 3
    TURN_RIGHT = 4

class SafeNavigation(Node):
    def __init__(self, robot_name):
        super().__init__('safe_navigation_node')

        # Keeping track of time based on number of cycles
        self.cycle_dt = 0.05   # 20Hz 
        self.cycle_current = 0
        self.cycle_last_transition = 0
        self.time_since_last_transition = 0

        qos_profile = QoSProfile(depth=10)

        # Subscribing to hazard detection data
        self.in_sub_hazard = self.create_subscription(
            HazardDetectionVector,
            f'/{robot_name}/hazard_detection',
            self.read_hazard,
            qos_profile_sensor_data
        )
        print("subscription in hazard done")
        # Publishing velocity commands
        self.out_pub_vel = self.create_publisher(
            Twist,
            f'/{robot_name}/cmd_vel',
            qos_profile
        )
        print("subscription in velocity done")

        # FSM initial state
        self.state = TurtleState.FORWARD

        # Hazard input placeholder
        self.hazard_detected = False
        self.hazard_direction = None  # Track the hazard direction

        # Timer to control state transitions and publishing
        self.timer = self.create_timer(self.cycle_dt, self.control_cycle)

    # Callback for hazard detection
    def read_hazard(self, msg):
        print("Processing hazard detection...")

        # Default values
        self.hazard_detected = False
        
        # Verifica se existe pelo menos uma detecção com o tipo 1 (colisão)
        for detection in msg.detections:
            print(detection)  # Debug: imprime o conteúdo de cada detecção

            if detection.type == 1:  # Colisão detectada
                self.hazard_detected = True
                frame = detection.header.frame_id
                print(f"⚠️ Collision at: {frame}")
                if frame == "bump_left" or frame == "bump_front_left":
                    self.hazard_direction = "left"
                    # print("self.hazard_direction setado como left")
                elif frame == "bump_right" or frame == "bump_front_right":
                    self.hazard_direction = "right"
                    # print("self.hazard_direction setado como right")
                elif frame == "bump_center" or frame == "bump_front_left":
                    self.hazard_direction = "front"
                    # print("self.hazard_direction setado como front")


    # Main control logic
    def control_cycle(self):
        if self.state == TurtleState.FORWARD:
            self.output_forward()
            # print("no control cycle indo para frente")

        elif self.state == TurtleState.BACKWARD:
            self.output_backward()
            # print("no control cycle indo para tras")

        elif self.state == TurtleState.TURN_LEFT:
            self.output_turn_left()
            # print("no control cycle indo para esquerda")

        elif self.state == TurtleState.TURN_RIGHT:
            self.output_turn_right()
            # print("no control cycle indo para direita")

        self.next_state()

    # Movement commands
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
        msg.angular.z = 0.5  # Turn left
        self.out_pub_vel.publish(msg)

    def output_turn_right(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -0.5  # Turn right
        self.out_pub_vel.publish(msg)

    def next_state(self):
        transition = False

        # print(f"Current state: {self.state}")

        if self.state == TurtleState.FORWARD:
            if self.hazard_detected:
                transition = True
                print(f"⚠️ HAZARD DETECTED: GO BACKWARD")
                self.state = TurtleState.BACKWARD

        elif self.state == TurtleState.BACKWARD:
            if self.evt_enough_time_spent(1.5):
                transition = True
                print(f"↪️ BACK DONE: GO TURN")
                if self.hazard_direction == "left":
                    self.state = TurtleState.TURN_RIGHT  # bateu na esquerda, vira à direita
                    # print("next_state indo para esquerda")
                elif self.hazard_direction == "right":
                    self.state = TurtleState.TURN_LEFT   # bateu na direita, vira à esquerda
                    # print("next_state indo para direita")
                elif self.hazard_direction == "front":
                    self.state = TurtleState.TURN_LEFT   # bateu de frente, pode virar para qualquer lado
                    # print("next_state indo para esquerda")

        elif self.state == TurtleState.TURN_LEFT or self.state == TurtleState.TURN_RIGHT:
            print("ENTROU NO IF DO STATE")
            if self.evt_enough_time_spent(1.5):
                transition = True
                print("➡️ TURN DONE: GO FORWARD")
                self.state = TurtleState.FORWARD

        self.cycle_current += 1
        if transition:
            self.cycle_last_transition = self.cycle_current
            self.time_since_last_transition = 0
        else:
            self.time_since_last_transition += self.cycle_dt

    # Timing event
    def evt_enough_time_spent(self, duration):
        return self.time_since_last_transition > duration

def main(args=None):
    rclpy.init(args=args)

    robot_name = sys.argv[1] if len(sys.argv) > 1 else input("Digite o nome do robô: ")
    print(f"Initializing SafeNavigation for {robot_name}...")

    node = SafeNavigation(robot_name)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
