#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <agiros_msgs/QuadState.h>

#include <std_msgs/Bool.h>

// Threshold for reaching the waypoint
const double PROXIMITY_THRESHOLD = 0.8;  // Adjust this value as needed

// UAV current position (to be updated via topic subscription)
double uav_x = 0.0, uav_y = 0.0, uav_z = 0.0;
bool waypoint_reached = true;
bool clustering_done = false;

// Locked target waypoint
double locked_x = 0.0, locked_y = 0.0, locked_z = 1.0;
double locked_yaw = 0.0;
bool target_locked = false;

// Publisher for sending waypoints
ros::Publisher waypoint_pub;


ros::Publisher waypoint_reached_pub;
ros::Publisher clustering_done_pub;



// Callback to update the UAV's current position
void uavPositionCallback(const agiros_msgs::QuadState::ConstPtr& msg) {
    uav_x = msg->pose.position.x;
    uav_y = msg->pose.position.y;
    uav_z = msg->pose.position.z;

    // ROS_WARN_STREAM("Updated UAV POS (QuadState): " << uav_x << ", " << uav_y << ", " << uav_z);
}

// Callback to receive the best waypoint
void bestWaypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // Only update the target if not locked
    if (!target_locked) {
        locked_x = msg->pose.position.x;
        locked_y = msg->pose.position.y;
        locked_z = msg->pose.position.z;

        // Extract yaw from quaternion
        double siny_cosp = 2 * (msg->pose.orientation.w * msg->pose.orientation.z);
        double cosy_cosp = 1 - 2 * (msg->pose.orientation.z * msg->pose.orientation.z);
        locked_yaw = atan2(siny_cosp, cosy_cosp);

        target_locked = true;
        ROS_WARN_STREAM("Locked Target: x=" << locked_x << ", y=" << locked_y << ", z=" << locked_z << ", yaw=" << locked_yaw);
    }

    // Calculate distance to locked target
    double distance_to_target = sqrt(pow(locked_x - uav_x, 2) + pow(locked_y - uav_y, 2));
    ROS_WARN_STREAM("UAV POS: " << uav_x << ", " << uav_y << ", " << uav_z);
    ROS_WARN_STREAM("Locked TARGET POS: " << locked_x << ", " << locked_y << ", " << locked_z);
    ROS_WARN_STREAM("Distance to Locked Target (x,y plane): " << distance_to_target);

    // Check if UAV is already at the locked target
    if (distance_to_target < PROXIMITY_THRESHOLD) {
        ROS_INFO("Waypoint reached. Waiting for next target...");
        waypoint_reached = true;
        target_locked = false;  // Unlock the target for the next update
        


        // Publish waypoint_reached status for clustering initialization
        std_msgs::Bool reached_msg;
        std_msgs::Bool clustering_done_msg;
        reached_msg.data = true;
        clustering_done_msg.data = false;
        waypoint_reached_pub.publish(reached_msg);
        clustering_done_pub.publish(clustering_done_msg);


        return;
    }

    // If the UAV is not at the locked target and hasn't been sent this waypoint yet
    if (waypoint_reached) {
        // Publish the locked waypoint to the UAV
        geometry_msgs::PoseStamped waypoint;
        waypoint.header.stamp = ros::Time::now();
        waypoint.header.frame_id = "world";
        
        waypoint.pose.position.x = locked_x;
        waypoint.pose.position.y = locked_y;
        // Ensure the UAV doesn't go below 0.7m altitude
        if (locked_z < 0.7) {
            locked_z = 0.7;
        }
        waypoint.pose.position.z = locked_z;

        // Set the orientation for yaw rotation
        waypoint.pose.orientation.x = 0.0;
        waypoint.pose.orientation.y = 0.0;
        waypoint.pose.orientation.z = sin(locked_yaw / 2);
        waypoint.pose.orientation.w = cos(locked_yaw / 2);

        ROS_INFO_STREAM("Publishing waypoint: x=" << locked_x << ", y=" << locked_y << ", z=" << locked_z << ", yaw=" << locked_yaw);
        waypoint_pub.publish(waypoint);

        waypoint_reached = false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_publisher");
    ros::NodeHandle nh;

    waypoint_reached_pub = nh.advertise<std_msgs::Bool>("/waypoint_reached", 10);
    clustering_done_pub = nh.advertise<std_msgs::Bool>("/clustering_done", 10);

    

    // Subscriber to UAV position
    ros::Subscriber uav_pos_sub = nh.subscribe<agiros_msgs::QuadState>("/kingfisher/agiros_pilot/state", 10, uavPositionCallback);

    // Subscriber to the best waypoint from frontier_detector.cpp
    ros::Subscriber waypoint_sub = nh.subscribe("/best_waypoint", 10, bestWaypointCallback);

    // Publisher for waypoints to Agilicious 
    // waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/kingfisher/agiros_pilot/go_to_pose", 10);
    // Publisher for waypoints to Ego-planner (to generate trajectory)
    waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_frontier_waypoint", 10);
    


    ros::spin();
    return 0;
}
