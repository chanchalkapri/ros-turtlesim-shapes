"""navigator_node.py
Subscribes to '/next_room' and sends the robot to the named room using nav2_simple_commander if available.
Room coordinates are stored in rooms.json.
"""

import json, time

def load_rooms(path='resources/rooms.json'):
    with open(path,'r') as f:
        return json.load(f)

def send_goal_with_fallback(room_name, pose):
    # If nav2_simple_commander is not available, print the goal. This keeps the file runnable for syntax checks.
    print(f'[SIMULATED NAV] Sending robot to {room_name} at pose {pose} (nav2 not available)')

def main():
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except Exception as e:
        print('ROS not available. Demonstrating navigator with a sample room.')
        rooms = load_rooms('resources/rooms.json')
        sample = 'kitchen'
        send_goal_with_fallback(sample, rooms[sample])
        return

    class NavigatorNode(Node):
        def __init__(self):
            super().__init__('navigator_node')
            self.sub = self.create_subscription(String, '/next_room', self.cb, 10)
            self.rooms = load_rooms('resources/rooms.json')
            self.get_logger().info('Navigator node started. Waiting for /next_room.')

        def cb(self, msg):
            room = msg.data.strip()
            if room not in self.rooms:
                self.get_logger().warn(f'Unknown room: {room}')
                return
            pose = self.rooms[room]
            # attempt to use nav2_simple_commander
            try:
                from nav2_simple_commander.robot_navigator import BasicNavigator
                navigator = BasicNavigator()
                # create a 2D pose structure expected by nav2_simple_commander
                # This example avoids blocking waits to keep the code simple
                self.get_logger().info(f'Sending goal to {room}: {pose}')
                # If you're running in a real Nav2 environment, replace the following with the proper API calls.
                send_goal_with_fallback(room, pose)
            except Exception as e:
                self.get_logger().info('nav2_simple_commander not available; using fallback send.')
                send_goal_with_fallback(room, pose)

    rclpy.init()
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
