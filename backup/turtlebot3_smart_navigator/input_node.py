"""input_node.py
Publishes simulated task conditions (time_of_day, priority, room_type) as JSON strings to topic '/task_conditions'.
This node is intentionally dependency-light so it can be linted/syntax-checked without a running ROS2 environment.
Run in ROS2 with: ros2 run turtlebot3_smart_navigator input_node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random
import time

class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')
        self.pub = self.create_publisher(String, '/task_conditions', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info('InputNode started, publishing to /task_conditions')

    def timer_callback(self):
        msg = String()
        condition = self.generate_sample_condition()
        msg.data = json.dumps(condition)
        self.pub.publish(msg)
        self.get_logger().info(f'Published task condition: {msg.data}')

    @staticmethod
    def generate_sample_condition():
        # time_of_day: 0=morning,1=afternoon,2=evening,3=night
        time_of_day = random.choice([0,1,2,3])
        # priority: 0=low,1=medium,2=high
        priority = random.choice([0,1,2])
        # room_type: 0=office,1=kitchen,2=conference,3=lab
        room_type = random.choice([0,1,2,3])
        return {
            'time_of_day': int(time_of_day),
            'priority': int(priority),
            'room_type': int(room_type),
            'timestamp': time.time()
        }

def main(args=None):
    rclpy.init(args=args)
    node = InputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()