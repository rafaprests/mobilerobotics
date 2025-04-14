import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import IrIntensityVector

class ChairAvoider(Node):
    def _init_(self):
        super()._init_('chair_avoider')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ir_sub = self.create_subscription(
            IrIntensityVector,
            '/ir_intensity',
            self.ir_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)
        self.obstacle_detected = False
        self.turning = False
        self.turn_ticks = 0

    def ir_callback(self, msg):
        self.obstacle_detected = False
        for reading in msg.readings:
            if reading.value > 70:  # adjust threshold if needed
                self.get_logger().info(f"Obstacle detected at {reading.header.frame_id}")
                self.obstacle_detected = True

    def control_loop(self):
        twist = Twist()

        if self.obstacle_detected and not self.turning:
            twist.linear.x = -0.1
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.5)

            twist.linear.x = 0.0
            twist.angular.z = 0.6
            self.cmd_vel_pub.publish(twist)
            self.turning = True
            self.turn_ticks = 5

        elif self.turning:
            if self.turn_ticks > 0:
                self.turn_ticks -= 1
            else:
                self.turning = False

        else:
            twist.linear.x = 0.15
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ChairAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()