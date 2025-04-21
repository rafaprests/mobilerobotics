# after achieving to do the moving part, we started trying to undock the robot

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import time

class MoveRobot(Node):
    def __init__(self, robot_name):
        super().__init__('move_robot')
        self.robot_name = robot_name
        self.cmd_vel_topic = f'/{robot_name}/cmd_vel'
        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic, 10)

    def undock(self):
        self.get_logger().info('Iniciando undock: movendo para trás...')
        msg = Twist()
        msg.linear.x = -0.2  # Move para trás a 0.2 m/s
        msg.angular.z = 0.0  # Sem rotação

        for _ in range(10):  # Move para trás por 2 segundos
            self.publisher_.publish(msg)
            time.sleep(0.2)

        self.get_logger().info('Undock concluído!')

def main(args=None):
    rclpy.init(args=args)

    robot_name = sys.argv[1] if len(sys.argv) > 1 else input("Type the name of the robot: ")

    move_robot = MoveRobot(robot_name)

    try:
        move_robot.undock()
    except KeyboardInterrupt:
        pass
    finally:
        move_robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
