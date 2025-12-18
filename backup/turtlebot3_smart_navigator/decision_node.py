"""decision_node.py
Subscribes to '/task_conditions' (JSON strings) and publishes '/next_room' (String with room name).
Decision flow:
  - Try to load models/model.pkl (sklearn-like) and use it if available.
  - Otherwise use fallback_rules.json or a simple heuristic.
Run with: ros2 run turtlebot3_smart_navigator decision_node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import traceback
from pathlib import Path

MODEL_PATH = Path(__file__).resolve().parents[1] / 'models' / 'model.pkl'
FALLBACK_RULES = Path(__file__).resolve().parents[1] / 'models' / 'fallback_rules.json'

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
        self.sub = self.create_subscription(String, '/task_conditions', self.on_task, 10)
        self.pub = self.create_publisher(String, '/next_room', 10)
        self.get_logger().info('DecisionNode started, waiting for /task_conditions')
        # Try to load fallback rules (a simple JSON) at startup
        self.rules = self.load_fallback_rules()
        # Model loading is optional
        self.model = self.try_load_model()

    def try_load_model(self):
        try:
            import joblib
            if MODEL_PATH.exists():
                model = joblib.load(MODEL_PATH)
                self.get_logger().info('Loaded ML model from models/model.pkl')
                return model
        except Exception as e:
            self.get_logger().info('ML model not available or failed to load; using fallback rules.')
            self.get_logger().debug(traceback.format_exc())
        return None

    def load_fallback_rules(self):
        try:
            if FALLBACK_RULES.exists():
                with open(FALLBACK_RULES, 'r') as fh:
                    rules = json.load(fh)
                    self.get_logger().info('Loaded fallback_rules.json')
                    return rules
        except Exception:
            self.get_logger().warning('Failed to load fallback_rules.json; using built-in defaults')
        # Default simple rules: mapping of (room_type, priority) -> room_name
        return {
            "default_map": {
                "0_0": "office_a",
                "0_1": "office_a",
                "0_2": "office_b",
                "1_0": "kitchen",
                "1_1": "kitchen",
                "1_2": "kitchen",
                "2_0": "conf_room",
                "2_1": "conf_room",
                "2_2": "conf_room",
                "3_0": "lab",
                "3_1": "lab",
                "3_2": "lab"
            }
        }

    def on_task(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            self.get_logger().warning('Received invalid JSON on /task_conditions')
            return
        next_room = self.decide_room(data)
        out = String()
        out.data = json.dumps({'room': next_room})
        self.pub.publish(out)
        self.get_logger().info(f'Decision made -> next_room: {next_room}')

    def decide_room(self, data):
        # If model available, try to use it
        try:
            if self.model is not None:
                # Expect the model to take features in order [time_of_day, priority, room_type]
                features = [data.get('time_of_day',0), data.get('priority',0), data.get('room_type',0)]
                prediction = self.model.predict([features])
                # model should return an index or string; handle both
                if isinstance(prediction, (list, tuple)) or hasattr(prediction, '__iter__'):
                    pred = prediction[0]
                else:
                    pred = prediction
                return str(pred)
        except Exception:
            self.get_logger().warning('Model inference failed; falling back to rules')
        # Fallback to rules
        try:
            key = f"{int(data.get('room_type',0))}_{int(data.get('priority',0))}"
            mapping = self.rules.get('default_map', {})
            return mapping.get(key, 'office_a')
        except Exception:
            return 'office_a'

def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()