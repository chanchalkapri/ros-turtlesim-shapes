# Simple ROS2 Python launch file for the package
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(Node(package='turtlebot3_smart_navigator', executable='input_node', name='input_node'))
    ld.add_action(Node(package='turtlebot3_smart_navigator', executable='decision_node', name='decision_node'))
    ld.add_action(Node(package='turtlebot3_smart_navigator', executable='navigator_node', name='navigator_node'))
    return ld