"""decision_node.py
Subscribes to '/task_conditions' (JSON string) and publishes the predicted room name to '/next_room'.
It attempts to load models/model.pkl. If it's a sklearn DecisionTreeClassifier it uses .predict(), otherwise
it supports a simple rules-based fallback saved in models/fallback_rules.json.
"""

import json

def load_model(path):
    try:
        import pickle
        with open(path,'rb') as f:
            model = pickle.load(f)
        return model
    except Exception as e:
        return None

def predict_with_fallback(model_obj, features):
    # model_obj could be a dict mapping simple thresholds to room indices
    if model_obj is None:
        # safe fallback: simple heuristic
        # priority high -> kitchen if morning/afternoon else living
        time_of_day, priority, room_type = features
        if priority == 2:
            return 'kitchen' if time_of_day in (0,1) else 'living_room'
        if room_type == 0:
            return 'bedroom'
        if room_type == 1:
            return 'kitchen'
        return 'living_room'
    # if model_obj has 'predict' attribute, assume sklearn-like
    if hasattr(model_obj, 'predict'):
        X = [[features[0], features[1], features[2]]]
        preds = model_obj.predict(X)
        mapping = {0:'bedroom',1:'kitchen',2:'living_room',3:'bathroom'}
        return mapping.get(int(preds[0]), 'living_room')
    # if it's a dict of rules
    if isinstance(model_obj, dict):
        # expect keys as strings like 'priority_2:time_0->kitchen'
        # try direct lookup by priority high
        key = f"priority_{features[1]}"
        if key in model_obj:
            return model_obj[key]
        # fallback heuristic
        return 'living_room'
    return 'living_room'

def main():
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except Exception as e:
        print('ROS not available. Demonstrating decision node prediction with sample inputs.')
        model = load_model('models/model.pkl')
        sample = {'time_of_day':0,'priority':2,'room_type':1}
        room = predict_with_fallback(model, (sample['time_of_day'], sample['priority'], sample['room_type']))
        print('Sample condition:', sample, '-> predicted room:', room)
        return

    class DecisionNode(Node):
        def __init__(self):
            super().__init__('decision_node')
            self.sub = self.create_subscription(String, '/task_conditions', self.cb, 10)
            self.pub = self.create_publisher(String, '/next_room', 10)
            self.model = load_model('models/model.pkl')
            self.get_logger().info('Decision node started, model loaded: %s' % (str(self.model)[:80]))

        def cb(self, msg):
            try:
                data = json.loads(msg.data)
                features = (data['time_of_day'], data['priority'], data['room_type'])
                room = predict_with_fallback(self.model, features)
                out = String()
                out.data = room
                self.pub.publish(out)
                self.get_logger().info(f'Predicted room: {room} from {data}')
            except Exception as e:
                self.get_logger().error('Failed to parse or predict: %s' % str(e))

    rclpy.init()
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
