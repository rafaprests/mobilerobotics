# 🛠️ Safe Navigation & Docking with iRobot Create 3

This ROS2 project implements a **safe navigation system** for the iRobot Create 3 robot, featuring autonomous movement, hazard detection, and a complete undocking and docking behavior.

## 🚀 Features

- 🧭 Random autonomous navigation using basic state transitions (forward, backward, turn left, turn right)
- ⚠️ Real-time hazard detection using bump sensors
- 🔌 Full undocking and docking using ROS2 action clients
- 🛰️ Odometry-based position tracking and orientation estimation (yaw angle)
- 🤖 Obstacle-avoidance logic during navigation and docking attempts

## 📦 Dependencies

This project is built using **ROS2 Iron** and requires the following packages:

- `rclpy`
- `geometry_msgs`
- `nav_msgs`
- `irobot_create_msgs`
- `rclpy.action`

You also need access to a **Create® 3** robot or simulator with topics and actions enabled for:
- `/cmd_vel`
- `/hazard_detection`
- `/odom`
- `/undock`
- `/dock`

## 📂 Project Structure

Main script:
```bash
safe_navigation.py
```

Core classes and logic:
- `SafeNavigation`: main ROS2 node with subscriptions, publishers, and action clients
- `TurtleState`: finite-state machine for robot movement
- Docking logic based on odometry and hazard awareness
- Real-time hazard monitoring
- Automatic recovery after collisions

## 🧠 How It Works

1. **Undock**: Starts by sending a goal to the undock action server.
2. **Random Navigation**: Moves around randomly while avoiding collisions using hazard data.
3. **Dock Navigation**: When instructed, it calculates the direction to the dock and navigates toward it using odometry.
4. **Docking**: Approaches the docking station and sends a goal to dock safely.
5. **Failsafe**: If a hazard is detected during docking, the robot aborts the docking and returns to random navigation for a short time before retrying.

## ⚙️ Usage

To run the script:
- run a docker image in your computer:
```bash
$ docker run -it --net=host --privileged --env="DISPLAY=$DISPLAY" --volume="${XAUTHORITY}:/root/.Xauthority" ros-iron-cyclone
```

- connect both robot and computer in the same network
- to confirm thay are in the same network, type the following command:
```bash
$ ros2 topic list
```
- if you are able to see the name of the robot and its topics, you are connected.

- once a container runs, the following command can be used to get its ID:
```bash
$ docker ps
```

- a new terminal can be connected to a container with ID using:
```bash
$ docker exec -it <id of container> bash
```

- copy your code to the container with:
```bash
$ docker cp move_robot.py <ID OF CONTAINER>:/root/
```

- once the code is copied, run with:
```bash
$ python3 name_of_your_code.py
```

## 🧪 Example Topics Used

- `/my_robot/hazard_detection`
- `/my_robot/odom`
- `/my_robot/cmd_vel`
- `/my_robot/dock`
- `/my_robot/undock`

## 🤝 Credits

Developed as part of a robotics navigation and control project using iRobot Create 3 and ROS2.