cmake_minimum_required(VERSION 3.0.2)
project(autonomous_exploration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  agiros_msgs
  # mavros
  # mavros_msgs
  geometry_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  octomap_ros
  octomap_msgs
)



find_package(mavlink REQUIRED)
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS perception/include  # Ensure perception headers are included
  CATKIN_DEPENDS roscpp agiros_msgs sensor_msgs pcl_ros pcl_conversions octomap_ros octomap_msgs 
)

catkin_install_python(PROGRAMS
  control/src/send_waypoint.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)





include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  # ${MAVLINK_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/perception/include  # Ensure correct include path
  ${PROJECT_SOURCE_DIR}/planning/include 
)

# Define the mapping node
# add_executable(mapping_node perception/src/mapping_node.cpp)
# target_link_libraries(mapping_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

# Define the frontier detection node
add_executable(frontier_detector planning/src/frontier_detector.cpp)
target_link_libraries(frontier_detector ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})


add_executable(waypoint_publisher planning/src/waypoint_publisher.cpp)
target_link_libraries(waypoint_publisher ${catkin_LIBRARIES})


add_executable(trajectory_publisher planning/src/trajectory_publisher.cpp)
target_link_libraries(trajectory_publisher ${catkin_LIBRARIES})

