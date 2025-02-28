#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <agiros_msgs/QuadState.h>

// Threshold for reaching the waypoint
const double PROXIMITY_THRESHOLD = 0.8;  // Adjust this value as needed

// UAV current position (to be updated via topic subscription)
double uav_x = 0.0, uav_y = 0.0, uav_z = 0.0;
bool waypoint_reached = true;

// Publisher for sending waypoints
ros::Publisher waypoint_pub;


void uavPositionCallback(const agiros_msgs::QuadState::ConstPtr& msg) {
    uav_x = msg->pose.position.x;
    uav_y = msg->pose.position.y;
    uav_z = msg->pose.position.z;

    // ROS_WARN_STREAM("Updated UAV POS (QuadState): " << uav_x << ", " << uav_y << ", " << uav_z);
}


// CURRENT ISSUE: target waypoints come in faster than the UAV can reach them. Lock target until UAV reaches it???? recalculate target only once uav reached target


// Callback to receive the best waypoint
void bestWaypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    double target_x = msg->pose.position.x;
    double target_y = msg->pose.position.y;
    double target_z = msg->pose.position.z;

    // Extract yaw from quaternion
    double siny_cosp = 2 * (msg->pose.orientation.w * msg->pose.orientation.z);
    double cosy_cosp = 1 - 2 * (msg->pose.orientation.z * msg->pose.orientation.z);
    double target_yaw = atan2(siny_cosp, cosy_cosp);

    // Check if UAV is already at the target
    // double distance_to_target = sqrt(pow(target_x - uav_x, 2) + pow(target_y - uav_y, 2) + pow(target_z - uav_z, 2));
    double distance_to_target = sqrt(pow(target_x - uav_x, 2) + pow(target_y - uav_y, 2));
    ROS_WARN_STREAM("UAV POS: " << uav_x << ", " << uav_y << ", " << uav_z);
    ROS_WARN_STREAM("TARGET POS: " << target_x << ", " << target_y << ", " << target_z);
    ROS_WARN_STREAM("AAAAAAAAAAAAAAAAAAA DISTTT x,y plane: " << distance_to_target);
    if (distance_to_target < PROXIMITY_THRESHOLD) {
        ROS_INFO("Waypoint reached. Waiting for next target...");
        waypoint_reached = true;
        return;
    }

    // If the UAV is not at the target and hasn't been sent this waypoint yet
    if (waypoint_reached) {
        // Publish the waypoint to the UAV
        geometry_msgs::PoseStamped waypoint;
        waypoint.header.stamp = ros::Time::now();
        waypoint.header.frame_id = "world";
        
        waypoint.pose.position.x = target_x;
        waypoint.pose.position.y = target_y;
        // waypoint.pose.position.z = target_z;

        // Ensure the UAV doesn't go below 0.7m altitude
        if (target_z < 0.7) {
            target_z = 0.7;
        }
        waypoint.pose.position.z = target_z;

        // Set the orientation for yaw rotation
        waypoint.pose.orientation.x = 0.0;
        waypoint.pose.orientation.y = 0.0;
        waypoint.pose.orientation.z = sin(target_yaw / 2);
        waypoint.pose.orientation.w = cos(target_yaw / 2);

        // ROS_INFO_STREAM("Publishing waypoint: x=" << target_x << ", y=" << target_y << ", z=" << target_z << ", yaw=" << target_yaw);
        waypoint_pub.publish(waypoint);

        waypoint_reached = false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_publisher");
    ros::NodeHandle nh;

    // Subscriber to UAV position
    // ros::Subscriber uav_pos_sub = nh.subscribe<agiros_msgs::QuadState>("/kingfisher/agiros_pilot/state", 10, uavPositionCallback);
    ros::Subscriber uav_pos_sub = nh.subscribe("/kingfisher/agiros_pilot/state", 10, uavPositionCallback);

    // Subscriber to the best waypoint from frontier_detector.cpp
    ros::Subscriber waypoint_sub = nh.subscribe("/best_waypoint", 10, bestWaypointCallback);

    // Publisher for waypoints
    waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/kingfisher/agiros_pilot/go_to_pose", 10);

    ros::spin();
    return 0;
}