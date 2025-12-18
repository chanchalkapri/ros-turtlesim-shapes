"""Navigator node for the smart navigator.

Listens for '/next_room' messages, looks up coordinates from resources/rooms.json,
and issues navigation goals. If the Nav2 simple commander is available the node will use it;
otherwise it publishes a PoseStamped message to '/goal_pose' as a simulation fallback.
"""

from __future__ import annotations

import json
import traceback
from pathlib import Path
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

BASE_DIR = Path(__file__).resolve().parents[1]
ROOMS_FILE = BASE_DIR / 'resources' / 'rooms.json'


class NavigatorNode(Node):
    """Node that translates chosen room names into navigation goals."""

    def __init__(self) -> None:
        super().__init__('navigator_node')
        self.subscription = self.create_subscription(String, '/next_room', self._on_next_room, 10)
        self.pose_publisher = self._create_pose_publisher()
        self.rooms = self._load_rooms()
        self.navigator = self._detect_nav2()
        self.get_logger().info('NavigatorNode ready.')

    def _create_pose_publisher(self):
        try:
            from geometry_msgs.msg import PoseStamped
            return self.create_publisher(PoseStamped, '/goal_pose', 10)
        except Exception:
            self.get_logger().debug('geometry_msgs not available; fallback publisher disabled.')
            return None

    def _load_rooms(self) -> Dict[str, Dict]:
        if ROOMS_FILE.exists():
            try:
                with ROOMS_FILE.open('r', encoding='utf-8') as fh:
                    return json.load(fh)
            except Exception:
                self.get_logger().warning('Failed to parse rooms.json; proceeding with empty map.')
                self.get_logger().debug(traceback.format_exc())
        return {}

    def _detect_nav2(self) -> Optional[object]:
        try:
            from nav2_simple_commander.robot_navigator import BasicNavigator  # type: ignore
            nav = BasicNavigator()
            self.get_logger().info('Nav2 simple commander detected and initialized.')
            return nav
        except Exception:
            self.get_logger().info('Nav2 simple commander not present; using fallback behavior.')
            self.get_logger().debug(traceback.format_exc())
            return None

    def _on_next_room(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            room = payload.get('room')
        except Exception:
            self.get_logger().warning('Received malformed JSON on /next_room. Ignoring.')
            return

        if not room:
            self.get_logger().warning('No room provided in /next_room message.')
            return

        coords = self.rooms.get(room)
        if coords is None:
            self.get_logger().warning('Unknown room: %s', str(room))
            return

        if self.navigator is not None:
            try:
                pose = self._coords_to_nav_pose(coords)
                self.get_logger().info('Sending nav2 goal to %s', room)
                self.navigator.goToPose(pose)
                return
            except Exception:
                self.get_logger().warning('Failed to send nav2 goal; falling back to publisher.')
                self.get_logger().debug(traceback.format_exc())

        # fallback: publish PoseStamped if possible
        if self.pose_publisher is not None:
            try:
                ps = self._coords_to_pose_stamped(coords)
                self.pose_publisher.publish(ps)
                self.get_logger().info('Published pose to /goal_pose for %s', room)
            except Exception:
                self.get_logger().warning('Failed to publish PoseStamped fallback.')
                self.get_logger().debug(traceback.format_exc())
        else:
            self.get_logger().info('Would command robot to %s at %s (fallback logging).', room, coords)

    @staticmethod
    def _coords_to_nav_pose(coords: Dict) -> Dict:
        return {
            'position': {'x': float(coords.get('x', 0.0)), 'y': float(coords.get('y', 0.0)), 'z': 0.0},
            'orientation': {'z': float(coords.get('yaw', 0.0)), 'w': 1.0},
        }

    @staticmethod
    def _coords_to_pose_stamped(coords: Dict):
        from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
        from rclpy.time import Time
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = Time().now().to_msg()
        ps.pose = Pose()
        ps.pose.position = Point(x=float(coords.get('x', 0.0)), y=float(coords.get('y', 0.0)), z=0.0)
        ps.pose.orientation = Quaternion(x=0.0, y=0.0, z=float(coords.get('yaw', 0.0)), w=1.0)
        return ps


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()