# this code doesnt work, it was our first try to implement the docking, our first
# strategy was to implement it separated from the main code, but then we changed it


import rclpy
from rclpy.node import Node
from irobot_create_msgs.action import Dock
from rclpy.action import ActionClient
import sys

class DockingNode(Node):
    def __init__(self, robot_name):
        super().__init__('dock_node')
        self.robot_name = robot_name
        self.dock_client = ActionClient(self, Dock, f'{robot_name}/dock')

    def send_dock_goal(self):
        self.get_logger().info(f'Conectando ao servidor de ação {self.robot_name}/dock...')
        self.dock_client.wait_for_server()

        goal_msg = Dock.Goal()
        self.get_logger().info('📡 Enviando goal de docking...')
        send_goal_future = self.dock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal de docking rejeitado.')
            return

        self.get_logger().info('✅ Goal aceito. Aguardando resultado...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result().result
        status = get_result_future.result().status

        if status == 4:  # 4 = SUCCESS
            self.get_logger().info('🚀 Docking realizado com sucesso!')
        else:
            self.get_logger().warn(f'⚠️ Docking falhou. Status: {status}')


def main(args=None):
    rclpy.init(args=args)

    robot_name = sys.argv[1] if len(sys.argv) > 1 else input("Type the name of the robot: ")


    node = DockingNode(robot_name)
    node.send_dock_goal()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
