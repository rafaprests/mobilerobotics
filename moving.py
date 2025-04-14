import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import IrIntensityVector
import sys

class RobotAvoidance(Node):
    def __init__(self, robot_name):
        super().__init__('robot_avoidance')
        
        # Create publisher for cmd_vel
        cmd_vel_topic = f'/{robot_name}/cmd_vel'
        self.publisher_ = self.create_publisher(Twist, cmd_vel_topic, 10)
        
        # Create subscriber for IR sensor data
        ir_topic = f'/{robot_name}/ir_intensity'
        self.ir_subscriber = self.create_subscription(
            IrIntensityVector,
            ir_topic,
            self.ir_callback,
            10
        )
        
        self.timer = self.create_timer(1.0, self.move_forward)  # Timer to move forward periodically
        self.obstacle_detected = False
        self.turning = False
        self.turn_ticks = 0
        self.robot_name = robot_name

    def ir_callback(self, msg):
        """ Callback to handle IR sensor data """
        self.obstacle_detected = False
        for reading in msg.readings:
            if reading.value > 70:  # Threshold for detecting obstacle
                self.get_logger().info(f"Obstacle detected at {reading.header.frame_id}")
                self.obstacle_detected = True

    def move_forward(self):
        """ Move robot forward or perform avoidance behavior if an obstacle is detected """
        twist = Twist()

        if self.obstacle_detected and not self.turning:
            self.get_logger().info("Obstacle detected! Avoiding...")

            # Move backward if obstacle is detected
            twist.linear.x = -0.2  # Move backward with a speed of 0.2 m/s
            self.publisher_.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.5)  # Give some time for robot to move backward
            
            # Stop and turn after backing up
            twist.linear.x = 0.0
            twist.angular.z = 0.6  # Rotate clockwise
            self.publisher_.publish(twist)
            self.turning = True
            self.turn_ticks = 5  # Set number of turns

        elif self.turning:
            if self.turn_ticks > 0:
                self.turn_ticks -= 1
            else:
                # After turning, move forward again
                self.turning = False
                twist.linear.x = 0.5  # Move forward with a speed of 0.5 m/s
                twist.angular.z = 0.0  # Stop rotating
                self.publisher_.publish(twist)
        else:
            # Move forward if no obstacle detected
            twist.linear.x = 0.5  # Move forward at 0.5 m/s
            twist.angular.z = 0.0  # No rotation
            self.publisher_.publish(twist)

    def stop_robot(self):
        """ Stop the robot """
        twist = Twist()
        self.publisher_.publish(twist)
        self.get_logger().info('Robot stopped.')

def main(args=None):
    rclpy.init(args=args)

    # Get the robot name via command-line argument or user input
    robot_name = sys.argv[1] if len(sys.argv) > 1 else input("Digite o nome do robô: ")

    # Initialize the RobotAvoidance node
    robot_avoidance = RobotAvoidance(robot_name)

    try:
        rclpy.spin(robot_avoidance)
    except KeyboardInterrupt:
        pass
    finally:
        robot_avoidance.stop_robot()  # Stop robot when exiting
        robot_avoidance.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()