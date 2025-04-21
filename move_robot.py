#first code of all, developed to understand how the publisher works

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

class MoveRobot(Node):
    def __init__(self, robot_name):
        super().__init__('move_robot')
        cmd_vel_topic = f'/{robot_name}/cmd_vel'
        self.publisher_ = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0, self.move_forward)

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.5  # Move para frente a 0.5 m/s
        msg.angular.z = 0.0  # Sem rotação
        self.publisher_.publish(msg)
        self.get_logger().info('Movendo o robô para frente...')

def main(args=None):
    rclpy.init(args=args)

    # Obtém o nome do robô via argumento de linha de comando ou entrada do usuário
    robot_name = sys.argv[1] if len(sys.argv) > 1 else input("Digite o nome do robô: ")

    move_robot = MoveRobot(robot_name)

    try:
        rclpy.spin(move_robot)
    except KeyboardInterrupt:
        pass
    finally:
        move_robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
