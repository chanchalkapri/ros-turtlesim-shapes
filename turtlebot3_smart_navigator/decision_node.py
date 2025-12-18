"""Decision node for the smart navigator.

This node subscribes to '/task_conditions' and publishes the chosen room to '/next_room'.
The selection is made using an optional ML model (if provided) or using simple fallback rules.
"""

from __future__ import annotations

import json
import traceback
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

BASE_DIR = Path(__file__).resolve().parents[1]
MODEL_FILE = BASE_DIR / 'models' / 'model.pkl'
RULES_FILE = BASE_DIR / 'models' / 'fallback_rules.json'


class DecisionNode(Node):
    """Node that decides the next room based on task conditions."""

    def __init__(self) -> None:
        super().__init__('decision_node')
        self.subscription = self.create_subscription(String, '/task_conditions', self._on_task, 10)
        self.publisher = self.create_publisher(String, '/next_room', 10)
        self.get_logger().info('DecisionNode ready and waiting for task conditions.')
        self.rules = self._load_rules()
        self.model = self._load_model()

    def _load_model(self) -> Optional[Any]:
        """Attempt to load a pretrained model (joblib format). Returns None when unavailable."""
        try:
            import joblib  # type: ignore
            if MODEL_FILE.exists():
                model = joblib.load(MODEL_FILE)
                self.get_logger().info('Model loaded from %s', str(MODEL_FILE))
                return model
            self.get_logger().debug('Model file not found at %s', str(MODEL_FILE))
        except Exception:
            self.get_logger().warning('Failed to load model; will use fallback rules.')
            self.get_logger().debug(traceback.format_exc())
        return None

    def _load_rules(self) -> Dict[str, Any]:
        """Load fallback rules from JSON, or return default mapping."""
        if RULES_FILE.exists():
            try:
                with RULES_FILE.open('r', encoding='utf-8') as fh:
                    rules = json.load(fh)
                    self.get_logger().info('Loaded fallback rules from %s', str(RULES_FILE))
                    return rules
            except Exception:
                self.get_logger().warning('Error reading fallback_rules.json; using defaults.')
                self.get_logger().debug(traceback.format_exc())

        # sensible default mapping: 'room_type_priority' -> room name
        return {
            'default_map': {
                '0_0': 'office_a',
                '0_1': 'office_a',
                '0_2': 'office_b',
                '1_0': 'kitchen',
                '1_1': 'kitchen',
                '1_2': 'kitchen',
                '2_0': 'conf_room',
                '2_1': 'conf_room',
                '2_2': 'conf_room',
                '3_0': 'lab',
                '3_1': 'lab',
                '3_2': 'lab',
            }
        }

    def _on_task(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            self.get_logger().warning('Received malformed JSON on /task_conditions. Ignoring.')
            return

        chosen = self._decide(payload)
        out = String()
        out.data = json.dumps({'room': chosen})
        self.publisher.publish(out)
        self.get_logger().info('Published next_room: %s', chosen)

    def _decide(self, data: Dict[str, Any]) -> str:
        """Decide which room to return for a task condition dict."""
        # Prefer model if available
        if self.model is not None:
            try:
                features = [int(data.get('time_of_day', 0)), int(data.get('priority', 0)), int(data.get('room_type', 0))]
                prediction = self.model.predict([features])
                if hasattr(prediction, '__iter__'):
                    value = prediction[0]
                else:
                    value = prediction
                return str(value)
            except Exception:
                self.get_logger().warning('Model inference failed; falling back to rules.')
                self.get_logger().debug(traceback.format_exc())

        # Use fallback rules
        try:
            key = f"{int(data.get('room_type', 0))}_{int(data.get('priority', 0))}"
            return self.rules.get('default_map', {}).get(key, 'office_a')
        except Exception:
            return 'office_a'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()