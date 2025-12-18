"""Input node for the smart navigator.

Periodically publishes simulated task conditions to the '/task_conditions' topic.
Each published message is a JSON string describing the task, for example:
{"time_of_day": 0, "priority": 2, "room_type": 1, "timestamp": 1690000000.0}
"""

from __future__ import annotations

import json
import random
import time
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class InputNode(Node):
    """Node that publishes simulated task conditions for testing the navigator."""

    def __init__(self) -> None:
        super().__init__(node_name='input_node')
        self.publisher = self.create_publisher(String, '/task_conditions', 10)
        self.timer = self.create_timer(2.0, self._on_timer)
        self.get_logger().info('InputNode initialized and publishing to /task_conditions')

    def _on_timer(self) -> None:
        message = String()
        payload = self._generate_condition()
        message.data = json.dumps(payload)
        self.publisher.publish(message)
        self.get_logger().debug('Published task condition: %s', message.data)

    @staticmethod
    def _generate_condition() -> Dict:
        """Generate a random task condition.

        Returns:
            dict: keys are time_of_day, priority, room_type, timestamp.
        """
        return {
            'time_of_day': int(random.choice([0, 1, 2, 3])),
            'priority': int(random.choice([0, 1, 2])),
            'room_type': int(random.choice([0, 1, 2, 3])),
            'timestamp': time.time(),
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()