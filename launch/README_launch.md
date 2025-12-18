Launch instructions (example):

1. Launch Gazebo house world and SLAM (turtlebot3_gazebo package):
   ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py  # or use your usual turtlebot3 launch

2. Run SLAM and save the map (use map_saver from nav2_map_server)

3. Launch Nav2 with the saved map and bringup launch files per Nav2 docs.

4. Launch the nodes (after sourcing workspace):
   ros2 run turtlebot3_smart_navigator input_node
   ros2 run turtlebot3_smart_navigator decision_node
   ros2 run turtlebot3_smart_navigator navigator_node
