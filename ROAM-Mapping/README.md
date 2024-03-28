# ROAM-Mapping
Distributed multi-robot semantic octree mapping implementation for building probabilistic multi-class maps of an environment using semantic pointcloud input and peer-to-peer communication.

## Dependencies
* ROS 1
* Catkin
* PCL
* OctoMap
* octomap_rviz_plugins
* ([SSMI-Mapping](https://github.com/ExistentialRobotics/SSMI/tree/main/SSMI-Mapping))

## How to use
1. `launch/semantic_octomap_multi.launch` starts ROS nodes `semantic_cloud` and `semantic_octomap_multi`.
2. `semantic_cloud` takes three **aligned** images, i.e. RBG image, depth image, and semantic segmentation image. The output is a sematic pointcloud topic of type `sensor_msgs/PointCloud2`.
3. `semantic_octomap_multi` receives the generated semantic pointcloud and updates the semantic OctoMap. Also, this node subscribes to a general topic named `/octomap_full` that is shared among all robots. The topic `/octomap_full` is communication channel for exchanging local semantic octree maps, which is used for reaching consensus in distributed mapping. The node `semantic_octomap_multi` internally maintains a semantic OcTree where every node stores the probability of each object class. However, only the maximmum likelihood semantic OcTree (with probabilistic occupancies) is published as an `octomap_msgs/Octomap` message, as well as a probabilistic 2-D occupancy map projected on the ground plane.
4. `params/multi_robot_semantic_mapping_params.yaml` stores camera intrinsic parameters, as well as the parameters for the distributed semantic octree mapping such as minimum grid size (`resolution`) and log-odds increments (`psi` and `phi`). Note that the camera intrinsic parameters should be set to the values used by your camera.
