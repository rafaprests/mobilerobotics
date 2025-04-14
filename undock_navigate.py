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
#from geometry_msgs.msg import PoseStamped


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

        self.in_sub_hazard = self.create_subscription(
            HazardDetectionVector,
            f'/{robot_name}/hazard_detection',
            self.read_hazard,
            qos_profile_sensor_data
        )
        self.out_pub_vel = self.create_publisher(
            Twist,
            f'/{robot_name}/cmd_vel',
            qos_profile
        )

        self.undock_client = ActionClient(self, Undock, f'/{self.robot_name}/undock')
        self.dock_client = ActionClient(self, Dock, f'/{self.robot_name}/dock')

        self.state = TurtleState.FORWARD
        self.hazard_detected = False
        self.hazard_direction = None

        self.timer = self.create_timer(self.cycle_dt, self.control_cycle)

    def undock(self):
        self.get_logger().info('Iniciando undock...')
        
        # Verifica se o servidor de undocking está disponível
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Servidor de undocking não está disponível')
            return
        
        goal_msg = Undock.Goal()
        send_goal_future = self.undock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Requisição de undocking rejeitada')
            return

        self.get_logger().info('⏳ Undocking em andamento...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info('✅ Undocking concluído com sucesso!')

    def undockManual(self):
        self.get_logger().info('Iniciando undock: movendo para trás...')
        msg = Twist()
        msg.linear.x = -0.2  # Move para trás a 0.2 m/s
        msg.angular.z = 0.0  # Sem rotação

        for _ in range(10):  # Move para trás por 2 segundos (10 * 0.2s)
            self.out_pub_vel.publish(msg)
            time.sleep(0.2)

        # Gira 180 graus
        self.get_logger().info('Realizando giro de 180 graus...')
        msg.linear.x = 0.0
        msg.angular.z = 0.5  # Velocidade angular positiva

        duration = 3.14 / 0.5  # Tempo para girar 180 graus = pi rad / velocidade
        steps = int(duration / 0.2)

        for _ in range(steps):  # Publica comandos por aproximadamente 6.28 segundos
            self.out_pub_vel.publish(msg)
            time.sleep(0.2)

        # Para o movimento
        self.get_logger().info('Giro completo. Undock finalizado.')
        msg.angular.z = 0.0
        self.out_pub_vel.publish(msg)

    def dock(self):
        self.get_logger().info('🔋 Iniciando docking...')

        if not self.dock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Servidor de docking não está disponível')
            return

        goal_msg = Dock.Goal()
        send_goal_future = self.dock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Requisição de docking rejeitada')
            return

        self.get_logger().info('⏳ Docking em andamento...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info('✅ Docking concluído com sucesso!')

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
        msg.angular.z = 0.5
        self.out_pub_vel.publish(msg)

    def output_turn_right(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -0.5
        self.out_pub_vel.publish(msg)

    def next_state(self):
        transition = False
        if self.state == TurtleState.FORWARD and self.hazard_detected:
            self.state = TurtleState.BACKWARD
            transition = True
        elif self.state == TurtleState.BACKWARD and self.evt_enough_time_spent(1.5):
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

    robot_name = sys.argv[1] if len(sys.argv) > 1 else input("Digite o nome do robô: ")
    print(f"Inicializando SafeNavigationDock para {robot_name}...")

    node = SafeNavigation(robot_name)

    try:
        # Etapa 1: undock + giro 180°
        node.undock()

        # Etapa 2: navega desviando obstáculos por 20 segundos
        print("Navegando por 20 segundos antes do docking...")
        start_time = time.time()
        while time.time() - start_time < 20:
            rclpy.spin_once(node)

        # Etapa 3: docking automático
        node.dock()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
