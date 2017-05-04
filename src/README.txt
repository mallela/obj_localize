1. Launch Gazebo and Rviz
  $ roslaunch turtlebot_octomap turtlebot_build_map.launch 

2. Launch teleop node
  $ roslaunch turtlebot_teleop keyboard_teleop.launch

3. load the object detector node - it detects the brown shelf. It finds the location of the shelf wrt the world (/map) and publishes it on to /obj_detect/location
  $ rosrun detec_obj_pkg detec_obj_pkg_node

4. Run the send_node_goal to allow robot to move to the goal ( at object location area)
  $ rosrun detec_obj_pkg detec_obj_pkg_node

5. publish 1 onto the /object_detect/sendGoal topic to send the goal value published by the detect_obj_pkg_node to send_goal_node 
  $ rostopic pub /object_detect/sendGoalag std_msgs/Int16 1



Debugging - if tranform between map and other frames is not found, run
  $ rosrun tf static_transform_publisher 0 0 0 0 0 1 map my_frame 10
