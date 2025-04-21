# second code, as oriented by the teacher, it would be easier for us if the robot moved by 
# a state machine, then we implemented this way

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
import time
from turtlesim.msg import Pose
from turtlesim.msg import Color

from enum import Enum

# Enumerating the states of the FSM
class TurtleState(Enum):
    FORWARD = 1
    BACKWARD = 2
    TURN = 3

class SafeNavigation(Node):
    def __init__(self, robot_name):
        super().__init__('move')

        # Keeping track of time based on number of cycles
        self.cycle_dt = 0.05   # 20Hz 
        self.cycle_current = 0
        self.cycle_last_transition = 0
        self.time_since_last_transition = 0


        ####### acho que eh nessa parte que tem que mudar algo
        # The FSM inputs are color_scan and Pose  
        qos_profile = QoSProfile(depth=10)
        self.in_sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.read_pose, qos_profile)
        self.in_sub_color = self.create_subscription(Color, '/turtle1/color_sensor', self.read_color, qos_profile)
        
        # The output of the FSM is cmd_vel
        self.out_pub_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', qos_profile)

        # Initialize state variables
        self.state = TurtleState.FORWARD
        self.in_pose = Pose()
        self.in_color = Color()

        # Timer to control state transitions and publishing
        self.timer = self.create_timer(self.cycle_dt, self.control_cycle)

    # Callbacks for inputs
    def read_pose(self,msg):
        self.in_pose = msg

    def read_color(self, msg):
        self.in_color = msg

    # Main function, computes output and calls transition function for next cycle
    def control_cycle(self):
        
        # Publish output depending on state
        if self.state == TurtleState.FORWARD:
            self.output_forward()

        elif self.state == TurtleState.BACKWARD:
            self.output_backward()

        elif self.state == TurtleState.TURN:
            self.output_turn()

        self.next_state()   

    def output_forward(self):
        msg = Twist()
        msg.linear.x = 5.0  # Move forward, fast 
        msg.angular.z = 0.0
        self.out_pub_vel.publish(msg)

    def output_backward(self):
        msg = Twist()
        msg.linear.x = -1.0  # Move back
        msg.angular.z = 0.0
        self.out_pub_vel.publish(msg)

    def output_turn(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 1.0  # Turn 
        self.out_pub_vel.publish(msg)

# Transition function
    def next_state(self):
        transition = False
        
        # Handle possible transitions
        if self.state == TurtleState.FORWARD:
            if self.evt_out_of_bound():
                transition = True
                print("GO BACKWARD")            
                self.state = TurtleState.BACKWARD

        elif self.state == TurtleState.BACKWARD:
            if self.evt_enough_time_spent(1):
                transition = True
                print("GO TURN")                  
                self.state = TurtleState.TURN
        
        elif self.state == TurtleState.TURN:
            if self.evt_enough_time_spent(1):
                transition = True
                print("GO FORWARD")                  
                self.state = TurtleState.FORWARD
        
        # Update cycle count
        self.cycle_current +=1
        if transition:
            self.cycle_last_transition = self.cycle_current
            self.time_since_last_transition = 0
        else:
            self.time_since_last_transition += self.cycle_dt
       
    # Events: return booleans when true
    def evt_out_of_bound(self):
        x = self.in_pose.x
        y = self.in_pose.y
        x_is_out = x<1 or x>10
        y_is_out = y<1 or y>10
        return x_is_out or y_is_out

    def evt_enough_time_spent(self, dur):
        return self.time_since_last_transition > dur

def main(args=None):
    rclpy.init(args=args)

    robot_name = sys.argv[1] if len(sys.argv) > 1 else input("Digite o nome do robô: ")
    print(f"Initializing SafeNavigation for {robot_name}...")  # Debugging log
    
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
