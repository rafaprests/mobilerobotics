import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import HazardDetectionVector
from enum import Enum

class TurtleState(Enum):
    FORWARD = 1
    BACKWARD = 2
    TURN = 3

class RobotFSM(Node):
    def __init__(self, robot_name):
        super().__init__('robot_fsm_node', namespace=robot_name)

        self.get_logger().info("Inicializando FSM...")

        # Publisher para enviar comandos de movimento
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber para detectar obstáculos
        self.subscription = self.create_subscription(
            HazardDetectionVector,
            'hazard_detection',
            self.hazard_callback,
            10
        )

        # Timer de controle
        self.timer = self.create_timer(0.1, self.control_cycle)

        # Estado atual da FSM
        self.state = TurtleState.FORWARD

        # Flags internas
        self.obstacle_detected = False
        self.last_obstacle = None

        # Controle de tempo
        self.cycle_duration = 0.1  # 10 Hz
        self.time_since_last_transition = 0.0

    def hazard_callback(self, msg):
        if not msg.detections:
            self.obstacle_detected = False
            self.last_obstacle = None
        else:
            self.obstacle_detected = True
            self.last_obstacle = msg.detections[0].header.frame_id
            self.get_logger().info(f"Obstáculo detectado: {self.last_obstacle}")

    def control_cycle(self):
        """Executa ações com base no estado atual"""
        if self.state == TurtleState.FORWARD:
            self.output_forward()
            if self.obstacle_detected:
                self.get_logger().info("Transição: FORWARD → BACKWARD")
                self.state = TurtleState.BACKWARD
                self.time_since_last_transition = 0.0

        elif self.state == TurtleState.BACKWARD:
            self.output_backward()
            self.time_since_last_transition += self.cycle_duration
            if self.time_since_last_transition > 1.0:
                self.get_logger().info("Transição: BACKWARD → TURN")
                self.state = TurtleState.TURN
                self.time_since_last_transition = 0.0

        elif self.state == TurtleState.TURN:
            self.output_turn()
            self.time_since_last_transition += self.cycle_duration
            if self.time_since_last_transition > 1.0:
                self.get_logger().info("Transição: TURN → FORWARD")
                self.state = TurtleState.FORWARD
                self.time_since_last_transition = 0.0

    def output_forward(self):
        cmd = Twist()
        cmd.linear.x = 0.2
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def output_backward(self):
        cmd = Twist()
        cmd.linear.x = -0.1
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def output_turn(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.6
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)

    robot_name = sys.argv[1] if len(sys.argv) > 1 else input("Digite o nome do robô: ")
    print(f"Inicializando FSM para {robot_name}...")

    node = RobotFSM(robot_name)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
