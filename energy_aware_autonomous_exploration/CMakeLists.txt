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
  project_msgs
  quadrotor_msgs
  std_msgs
  message_runtime
  message_generation
)




find_package(Python3 REQUIRED COMPONENTS Development NumPy)
include_directories(${Python3_INCLUDE_DIRS})
link_libraries(${Python3_LIBRARIES})




find_package(mavlink REQUIRED)
find_package(gazebo REQUIRED)
find_package(PCL REQUIRED)   # Ensure PCL is found
find_package(octomap REQUIRED)  # Ensure Octomap is found
find_package(yaml-cpp REQUIRED) # Ensure YAML-cpp is found


# add_service_files(
#   DIRECTORY planning/srv
#   FILES
#   Test.srv
# )

# add_service_files(
#   DIRECTORY local_planner/srv
#   FILES
#   GetTrajectory.srv
# )

# generate_messages(
#   DEPENDENCIES 
#   std_msgs
# )


catkin_package(
  INCLUDE_DIRS perception/include  # Ensure perception headers are included
  CATKIN_DEPENDS roscpp agiros_msgs sensor_msgs pcl_ros pcl_conversions octomap_ros octomap_msgs quadrotor_msgs std_msgs message_runtime message_generation
  LIBRARIES yaml-cpp
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
  ${PROJECT_SOURCE_DIR}/utilities/include

  ${PROJECT_SOURCE_DIR}/include

  ${YAML_CPP_INCLUDE_DIR}
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



add_executable(custom_octomap_server perception/src/custom_octomap_server.cpp)
target_link_libraries(custom_octomap_server ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})


add_executable(performance_tracker utilities/performance_tracker.cpp)
target_link_libraries(performance_tracker ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} pthread)


add_executable(entropy_calculator utilities/map_entropy_calculator.cpp)
target_link_libraries(entropy_calculator ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} pthread)


add_executable(energy_logger utilities/energy_logger.cpp)
target_link_libraries(energy_logger ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} pthread)


