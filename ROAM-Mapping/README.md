# ROAM-Mapping
Distributed multi-robot semantic octree mapping implementation for building probabilistic multi-class maps of an environment using semantic pointcloud input and peer-to-peer communication.

## Dependencies
* ROS 1
* Catkin
* PCL
* OctoMap
* octomap_rviz_plugins
* [SSMI-Mapping](https://github.com/ExistentialRobotics/SSMI/tree/main/SSMI-Mapping)

## How to use
1. `launch/semantic_octomap_multi.launch` starts ROS nodes `semantic_cloud` and `semantic_octomap_multi`.
2. `semantic_cloud` takes three **aligned** images, i.e. RBG image, depth image, and semantic segmentation image. The output is a sematic pointcloud topic of type `sensor_msgs/PointCloud2`.
3. `semantic_octomap_multi` receives the generated semantic pointcloud and updates the semantic OctoMap. This node internally maintains a semantic OcTree where each node stores the probability of each object class. Two types of semantic OctoMap topic are published as instances of `octomap_multi/Octomap_multi` message: `octomap_full` and `robot_name/octomap_color`. `octomap_full` contains the full probability distribution over the object classes, while `robot_name/octomap_color` only stores the maximmum likelihood semantic OcTree (with probabilistic occupancies). The topic `/octomap_full` is a communication medium for exchanging local semantic octree maps, which is used for reaching consensus in distributed mapping. A probabilistic 2-D occupancy map topic is additionally published via projection of the OctoMap on the ground plane.
4. Note that `octomap_rviz_plugins` can only visualize `robot_name/octomap_color`, whereas visualizing `octomap_full` causes Rviz to crash.
5. `params/multi_robot_semantic_mapping_params.yaml` stores camera intrinsic parameters, as well as the parameters for the distributed semantic octree mapping such as minimum grid size (`resolution`) and log-odds increments (`psi` and `phi`). Note that the camera intrinsic parameters should be set to the values used by your camera.
