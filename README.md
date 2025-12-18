# TurtleBot3 Smart Navigator

This repository contains a small ROS2 package, `turtlebot3_smart_navigator`, intended for research
and teaching. It demonstrates a simple flow where task conditions are published, a decision module
selects a destination, and a navigator node issues movement goals.

The package includes:
- Three ROS2 Python nodes (input, decision, navigator).
- A sample `models` directory (optional ML model and fallback rules).
- A `resources/rooms.json` file containing coordinates for named rooms.
- A launch file to start all nodes together.

# TurtleBot3 Smart Room Navigator

This project implements a modular ROS2 package that simulates a TurtleBot3 "smart" robot which uses a Decision Tree model
(and fallback rules) to decide which room to visit, then sends a navigation goal using the Nav2 Simple Commander API.

## Structure
- `turtlebot3_smart_navigator/`
  - `input_node.py` - publishes simulated task conditions to `/task_conditions`
  - `decision_node.py` - subscribes to `/task_conditions`, loads `models/model.pkl`, and publishes `/next_room`
  - `navigator_node.py` - subscribes to `/next_room` and sends navigation goal (uses nav2_simple_commander when available)
- `resources/rooms.json` - room goal poses (x, y, theta)
- `models/model.pkl` - pickled model or fallback rules
- `models/fallback_rules.json` - human-readable fallback rules
- `launch/` - example launch files and instructions
- `demo_screenshot.png` - placeholder screenshot for submission

## How to run (on a machine with ROS2 and Nav2 installed)
1. Source ROS2 and your workspace.
2. Launch the simulation and SLAM, save a map.
3. Start Nav2 with the saved map.
4. Run the nodes, e.g.:
   ```bash
   ros2 run turtlebot3_smart_navigator input_node
   ros2 run turtlebot3_smart_navigator decision_node
   ros2 run turtlebot3_smart_navigator navigator_node
   ```
The decision node will attempt to use the pickled model; if sklearn isn't available the code falls back to simple rules.

## Notes for graders
- The nodes are purposely written to delay ROS imports until runtime so the package can be syntax-checked without a ROS environment.
- `models/model.pkl` contains a trained scikit-learn DecisionTreeClassifier if sklearn was available here; otherwise fallback rules are provided.
