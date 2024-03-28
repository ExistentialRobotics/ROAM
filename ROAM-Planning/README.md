# ROAM-Planning
Implementation of distributed multi-robot planning for exploration. In order to ensure correct operation of the planner, you need to run ROAM-Mapping alongside this package so that the robots plan on the same glaboally consistant common map.

## Dependencies
* ROS Melodic
* Catkin
* [rospy_tutorials](https://github.com/ros/ros_tutorials/tree/noetic-devel/rospy_tutorials)
* pybind11

## How to use
1. `launch/planning_multi.launch` starts ROS nodes `ga_exploration` and `path_navigation_node` (optional).
2. `params/multi_robot_exploration_params.yaml` stores distributed planning parameters, such as planning horizon (`horizon`) and maximum number iterations (`max_iter`).
