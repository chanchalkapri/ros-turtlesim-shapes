"""navigator_node.py
Subscribes to '/next_room' (JSON string with {'room': 'room_name'}) and attempts to send navigation goals.
If nav2_simple_commander is available, uses it. Otherwise performs a safe simulated "fallback" action
(publishes to /goal_pose as a PoseStamped if geometry_msgs is present, otherwise logs).
Run with: ros2 run turtlebot3_smart_navigator navigator_node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from pathlib import Path
import traceback

ROOMS_FILE = Path(__file__).resolve().parents[1] / 'resources' / 'rooms.json'

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')
        self.sub = self.create_subscription(String, '/next_room', self.on_next_room, 10)
        self.get_logger().info('NavigatorNode started, waiting for /next_room')
        self.rooms = self.load_rooms()
        # Try to detect Nav2 simple commander
        self.nav = self.try_nav2()
        # If geometry msgs available, create a publisher to /goal_pose for simulation/fallback
        try:
            from geometry_msgs.msg import PoseStamped
            self.pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        except Exception:
            self.pose_pub = None

    def load_rooms(self):
        try:
            if ROOMS_FILE.exists():
                with open(ROOMS_FILE, 'r') as fh:
                    return json.load(fh)
        except Exception:
            self.get_logger().warning('Failed to load rooms.json; continuing with empty map')
        return {}

    def try_nav2(self):
        try:
            # This will fail if nav2_simple_commander is not installed; that's expected in CI/lint env
            from nav2_simple_commander.robot_navigator import BasicNavigator
            nav = BasicNavigator()
            self.get_logger().info('nav2_simple_commander available; ready to send nav2 goals')
            return nav
        except Exception:
            self.get_logger().info('nav2_simple_commander not available; will use fallback publisher or logging')
            self.get_logger().debug(traceback.format_exc())
            return None

    def on_next_room(self, msg):
        try:
            data = json.loads(msg.data)
            room = data.get('room')
        except Exception:
            self.get_logger().warning('Received invalid JSON on /next_room')
            return
        if not room:
            self.get_logger().warning('No room specified in /next_room message')
            return
        coords = self.rooms.get(room)
        if coords is None:
            self.get_logger().warning(f'Room "{room}" not found in rooms.json')
            return
        # If nav2 available, send nav2 goal
        if self.nav is not None:
            try:
                pose = self.coords_to_pose(coords)
                self.get_logger().info(f'Sending nav2 goal to {room} at {coords}')
                # BasicNavigator usage: go to pose (this is minimal; real usage requires full nav2 stack)
                self.nav.goToPose(pose)
                return
            except Exception:
                self.get_logger().warning('Failed to send nav2 goal; falling back to publisher/logging')
                self.get_logger().debug(traceback.format_exc())
        # Fallback: publish PoseStamped to /goal_pose if possible
        if self.pose_pub is not None:
            try:
                ps = self.coords_to_pose_stamped(coords)
                self.pose_pub.publish(ps)
                self.get_logger().info(f'Published PoseStamped fallback for {room}')
            except Exception:
                self.get_logger().warning('Failed to publish PoseStamped fallback')
                self.get_logger().debug(traceback.format_exc())
        else:
            self.get_logger().info(f'Fallback: would command robot to {room} at {coords}')

    @staticmethod
    def coords_to_pose(coords):
        # Compose a pose dictionary for nav2_simple_commander BasicNavigator (expecting a dict-like pose)
        # coords expected: {'x':..., 'y':..., 'yaw':...}
        return {
            'position': {'x': float(coords.get('x', 0.0)), 'y': float(coords.get('y', 0.0)), 'z': 0.0},
            'orientation': {'z': float(coords.get('yaw', 0.0)), 'w': 1.0}
        }

    @staticmethod
    def coords_to_pose_stamped(coords):
        from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = rclpy.clock.Clock().now().to_msg()
        ps.pose = Pose()
        ps.pose.position = Point(x=float(coords.get('x',0.0)), y=float(coords.get('y',0.0)), z=0.0)
        # Simple quaternion from yaw (approximate, w=1)
        ps.pose.orientation = Quaternion(x=0.0, y=0.0, z=float(coords.get('yaw',0.0)), w=1.0)
        return ps

def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()