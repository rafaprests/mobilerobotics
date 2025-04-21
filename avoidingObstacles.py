# in this code we managed to move the robot as the asked in the project. the final code
# was developed from this code

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from irobot_create_msgs.msg import HazardDetectionVector  # type: ignore # Para detectar obstáculos

class SafeNavigation(Node):
    def __init__(self):
        super().__init__('safe_navigation')

        # Criar um publisher para enviar comandos de movimento
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Criar um subscriber para detectar obstáculos
        self.create_subscription(HazardDetectionVector, '/hazard_detection', self.hazard_callback, 10)

        # Criar um timer para mover o robô periodicamente
        self.timer = self.create_timer(0.1, self.move_robot)

        # Estado do robô
        self.obstacle_detected = False
        self.last_obstacle = None

    def hazard_callback(self, msg):
        """Callback chamada quando há uma nova leitura do sensor de obstáculos"""
        if msg.detections:
            self.obstacle_detected = True
            self.last_obstacle = [det.header.frame_id for det in msg.detections]  # Pega todos os sensores detectados
        else:
            self.obstacle_detected = False
            self.last_obstacle = None

    def move_robot(self):
        """Controla o movimento do robô com base nos sensores"""
        cmd = Twist()

        if self.obstacle_detected:
            self.get_logger().info(f"Obstáculo detectado em {self.last_obstacle}")

            if "front_center" in self.last_obstacle:
                # Obstáculo na frente → Girar para evitar colisão
                cmd.angular.z = 1.0
                self.get_logger().info("Girando para evitar obstáculo à frente.")
            elif "front_right" in self.last_obstacle or "bump_right" in self.last_obstacle:
                # Obstáculo à direita → Virar um pouco à esquerda
                cmd.angular.z = 0.5
                self.get_logger().info("Obstáculo à direita, virando para a esquerda.")
            elif "front_left" in self.last_obstacle or "bump_left" in self.last_obstacle:
                # Obstáculo à esquerda → Virar um pouco à direita
                cmd.angular.z = -0.5
                self.get_logger().info("Obstáculo à esquerda, virando para a direita.")
            else:
                # Obstáculo desconhecido → Parar
                cmd.linear.x = 0.0
                self.get_logger().info("Obstáculo desconhecido, parando.")
        else:
            # Sem obstáculos → Seguir reto
            cmd.linear.x = 0.2
            self.get_logger().info("Seguindo em frente.")

        # Publicar comando
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    robot_name = sys.argv[1] if len(sys.argv) > 1 else input("Type the name of the robot: ")
    
    node = SafeNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
