# Energy-aware Autonomous Exploration for UAVs in Unknown Environments #
MSc thesis project by Jacob Elskamp




## Getting Started
Clone project into your `catkin` workspace and `catkin build --force-cmake`:
```
# Create new catkin workspace.
cd catkin_ws_exp/src

# Clone folder in workspace
git clone https://github.com/jelskamp/EAAE.git

catkin build --force-cmake
```

NEW:
Clone project into caktin ws

(TODO ensure ego_planner is in repo)

install dependencies:

ros-noetic-octomap*

ros-noetic-mavlink

$ sudo apt install libgoogle-glog-dev 
$ sudo apt-get install libgflags-dev
$ sudo apt install libgoogle-glog-dev
$ sudo apt-get install protobuf-compiler libprotobuf-dev
-------------------------















To launch simulation use agisim.launch. (This automatically starts gazebo, spawns kingfisher UAV including depth camera)

```
cd ~/catkin_ws_src
source devel/setup.bash
roslaunch agiros agisim.launch
```

To use Gazebo GUI: go to agisim.launch, <include .../base_quad_simulation.launch> and adjust: ```name="GUI" value="false" ```.

To launch mapping algorithm:
```
roslaunch autonomous_exploration mapping.launch
```

To start exploration:
```
roslaunch autonomous_exploration waypoint_publisher.launch
```



Send waypoint commands by:
```
rosrun autonomous_exploration send_waypoint.py
```
Go to send_waypoint.py to adjust waypoints or send a single waypoint, a sequence of waypoints, or a trajectory. 





Current known issues (to-be-fixed):
Warning msg when running agisim.launch & mapping.launch regarding double transform information (see below)
"TF_REPEATED_DATA ignoring data with redundant timestamp for frame kingfisher/base_link (parent world)"

