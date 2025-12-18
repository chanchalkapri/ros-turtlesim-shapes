"""input_node.py
Publishes simulated task conditions (time_of_day, priority, room_type) as JSON strings to topic '/task_conditions'.
This file is written to avoid top-level ROS imports so it can be syntax-checked without ROS present.
Run with: ros2 run turtlebot3_smart_navigator input_node
"""

import time, json, random

def generate_sample_condition():
    # time_of_day: 0=morning,1=afternoon,2=evening,3=night
    time_of_day = random.choice([0,1,2,3])
    # priority: 0=low,1=medium,2=high
    priority = random.choice([0,1,2])
    # room_type: 0=bedroom,1=kitchen,2=living,3=bathroom
    room_type = random.choice([0,1,2,3])
    return {'time_of_day': time_of_day, 'priority': priority, 'room_type': room_type}

def main():
    # Lazy import of rclpy so this file can be syntax-checked without ROS.
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except Exception as e:
        print('ROS not available in this environment. To run this node, source ROS2 and run with ros2 run.')
        # for submission purposes, just demonstrate generator output
        for _ in range(3):
            print(json.dumps(generate_sample_condition()))
            time.sleep(0.5)
        return

    class InputNode(Node):
        def __init__(self):
            super().__init__('input_node')
            self.pub = self.create_publisher(String, '/task_conditions', 10)
            self.timer = self.create_timer(1.0, self.timer_cb)
            self.get_logger().info('Input node started, publishing /task_conditions')

        def timer_cb(self):
            cond = generate_sample_condition()
            msg = String()
            msg.data = json.dumps(cond)
            self.pub.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')

    rclpy.init()
    node = InputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
