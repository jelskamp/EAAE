project(agiros)
cmake_minimum_required(VERSION 3.0.0)



find_package(catkin_simple REQUIRED)



# add_service_files(
#   DIRECTORY srv
#   FILES
#   getEnergy.srv
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   agiros_msgs
# )  


# FOR GAZEBO (ADDED BY ME) , ALSO SEE CMAKELISTS
find_package(gazebo REQUIRED)
# FOR TF BROADCASTING
find_package(catkin REQUIRED COMPONENTS
  agiros_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf
  std_msgs
  message_generation
)
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
#   geometry_msgs
#   nav_msgs
#   tf
#   agiros_msgs
#   message_generation
# )




catkin_simple(ALL_DEPS_REQUIRED)

# Setup Default Build Type as Release
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()

# Setup Architcture-specific Flags
if ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
    message(STATUS "Using ARMv7 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
elseif ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
    message(STATUS "Using ARM aarch64 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
elseif ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
    message(STATUS "Using ARM arm64 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
else ()
    if (NOT DEFINED CATKIN_DEVEL_PREFIX)
        set(CMAKE_CXX_ARCH_FLAGS " -march=native")
    endif ()
endif ()

# Setup General C++ Flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_STACK_ALLOCATION_LIMIT=1048576")

# Setup Release and Debug flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_ARCH_FLAGS} -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g")

# Define Library
cs_add_library(ros_pilot
        src/ros_pilot.cpp
        src/ros_traj_visualizer.cpp
        src/bridge/rotors_bridge.cpp
        src/bridge/ros_bridge.cpp
        src/ctrl_feedback_publisher.cpp
        )

        
      

# cs_add_service_files(
#   FILES
#   getEnergy.srv
# )

# cs_generate_messages()


# FOR GAZEBO (ADDED BY ME) , ALSO SEE CMAKELISTS
include_directories(
#  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
        
        
        
target_link_libraries(ros_pilot
  ${catkin_LIBRARIES}
	$<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.0>>:stdc++fs>)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  link_directories("${PROJECT_SOURCE_DIR}/../../agilib/externals/acados-src/lib") # Hack, necessary for now until we fix linking with gcc
endif()

target_link_libraries(ros_pilot "-Wl,--disable-new-dtags")

target_compile_options(ros_pilot PRIVATE
  -fno-finite-math-only
  -Wall
  -Werror
  -Wpedantic
  -Wunused
  -Wno-unused-parameter
  -Wundef
  -Wcast-align
  -Wredundant-decls
  -Wodr
  -Wunreachable-code
  -Wno-non-virtual-dtor
)

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  target_compile_options(ros_pilot PRIVATE
    -Wmissing-include-dirs
    -Wmissing-declarations
    # To keep the compiler calm about Eigen
    -Wno-sign-conversion
    -Wno-implicit-int-float-conversion
    -Wno-c99-extensions
    -Wno-implicit-int-conversion
    -Wno-non-virtual-dtor
  )
endif()


# Define Executable
cs_add_executable(${PROJECT_NAME}
        src/node.cpp)
target_link_libraries(${PROJECT_NAME}
        ros_pilot)


# Simulator stuff
cs_add_executable(agisim_node
        src/agisim_node.cpp)
target_link_libraries(agisim_node
        ros_pilot
        )
  
# ROS to Laird bridge example
cs_add_executable(ros_to_laird_bridge src/ros_to_laird_bridge.cpp)
target_link_libraries(ros_to_laird_bridge ros_pilot)

# ROS to MSP bridge example
cs_add_executable(ros_to_msp_bridge src/ros_to_msp_bridge.cpp)
target_link_libraries(ros_to_msp_bridge ros_pilot)

# Laird to SBUS bridge
cs_add_executable(laird_to_sbus_bridge src/laird_to_sbus_bridge.cpp)
target_link_libraries(laird_to_sbus_bridge ros_pilot)

# Laird to CTRL bridge
cs_add_executable(laird_to_ctrl_bridge src/laird_to_ctrl_bridge.cpp)
target_link_libraries(laird_to_ctrl_bridge ros_pilot)

# ROS to CTRL bridge
cs_add_executable(ros_to_ctrl_bridge src/ros_to_ctrl_bridge.cpp)
target_link_libraries(ros_to_ctrl_bridge ros_pilot)

# MockVio Node
cs_add_executable(mock_vio src/mockvio_node.cpp)
target_link_libraries(mock_vio ros_pilot)




# Add the UAV pose update plugin ADDED BY ME!!!!!!!!!!!!!
cs_add_library(update_uav_pose_plugin src/update_uav_pose_plugin.cpp)
target_link_libraries(update_uav_pose_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(update_uav_pose_plugin ${catkin_EXPORTED_TARGETS})


# Add uav TF broadcaster ADDED BY ME!!!!!!!!!!!!!
# cs_add_library(uav_tf_broadcast src/uav_tf_broadcast.cpp)
cs_add_executable(uav_tf_broadcast src/uav_tf_broadcast.cpp)
target_link_libraries(uav_tf_broadcast ${catkin_LIBRARIES})


# Add the odometry publisher node
cs_add_executable(odom_publisher_for_ego src/odom_publisher_for_ego.cpp)
target_link_libraries(odom_publisher_for_ego ${catkin_LIBRARIES})


# Energy Estimator Node - no service
# cs_add_executable(energy_estimator_node src/energy_estimator.cpp)
# target_link_libraries(energy_estimator_node ros_pilot)

cs_add_executable(energy_estimator_node_service src/energy_estimator.cpp)
target_link_libraries(energy_estimator_node_service ros_pilot)



# Finish
cs_install()
cs_export()
