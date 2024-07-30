cmake_minimum_required(VERSION 2.8)
project(roam_mapping)

set(CMAKE_BUILD_TYPE Release)

set(PACKAGE_DEPENDENCIES
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  octomap_msgs
  semantic_octomap
  message_generation
)

find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)

find_package(octomap REQUIRED)

find_package(semantic_octomap REQUIRED)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${semantic_octomap_INCLUDE_DIRS}
)

add_message_files(
  DIRECTORY msg
  FILES Octomap_multi.msg Named_float.msg
  )
  
generate_messages(DEPENDENCIES octomap_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS octomap semantic_octomap PCL message_runtime std_msgs
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${semantic_octomap_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Remove octomap_multi
add_executable(semantic_octomap_multi
 src/octomap_generator_semantic_ros.cpp
)

target_link_libraries(semantic_octomap_multi ${LINK_LIBS})

add_executable(compute_phi_node
 src/compute_phi.cpp
)

target_link_libraries(compute_phi_node ${LINK_LIBS})

add_executable(compute_entropy_node
 src/compute_entropy.cpp
)

target_link_libraries(compute_entropy_node ${LINK_LIBS})

add_executable(compute_coverage_node
 src/compute_coverage.cpp
)

target_link_libraries(compute_coverage_node ${LINK_LIBS})

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  )

install(DIRECTORY
  params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml"
  )
# Remove octimap_multi
install(TARGETS semantic_octomap_multi
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  )
