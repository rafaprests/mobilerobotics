import control.launch as launch
from control.launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot', description='Name of the robot'),
        DeclareLaunchArgument('config_file', default_value='twist_mux.yaml', description='Path to twist_mux YAML configuration file'),
        
        # Launch the twist_mux node
        launch_ros.actions.Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            remappings=[
                ('/cmd_vel', '/cmd_vel_mux')
            ]
        ),

        # Launch the SafeNavigation node
        launch_ros.actions.Node(
            package='your_package',  # Replace with your actual package name
            executable='taking_control',
            name='safe_navigation_node',
            output='screen',
            parameters=[{'robot_name': LaunchConfiguration('robot_name')}]
        ),
    ])
