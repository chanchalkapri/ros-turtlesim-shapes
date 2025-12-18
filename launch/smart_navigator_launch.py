"""Launch file to run the smart navigator nodes together."""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='turtlebot3_smart_navigator', executable='input_node', name='input_node'),
        Node(package='turtlebot3_smart_navigator', executable='decision_node', name='decision_node'),
        Node(package='turtlebot3_smart_navigator', executable='navigator_node', name='navigator_node'),
    ])