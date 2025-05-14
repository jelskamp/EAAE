#include <ros/ros.h>

#include <agiros_msgs/Reference.h>
#include <agiros_msgs/Setpoint.h>
#include <agiros_msgs/QuadState.h>
#include <agiros_msgs/Command.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>


// TODO : INCLUDE / MAKE ONE WORKSPACE
#include <quadrotor_msgs/PositionCommand.h>


ros::Publisher traj_pub;

// Function to normalize quaternion
geometry_msgs::Quaternion normalizeQuaternion(double yaw) {
    geometry_msgs::Quaternion q;
    q.w = cos(yaw / 2.0);
    q.z = sin(yaw / 2.0);
    q.x = 0.0;
    q.y = 0.0;
    return q;
}



// Callback function for trajectory messages from EGO-Planner
void trajectoryCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    agiros_msgs::Reference trajectory_msg;
    
    agiros_msgs::Setpoint setpoint;
    setpoint.state = agiros_msgs::QuadState();
    setpoint.command = agiros_msgs::Command();

    // Set timestamps
    setpoint.state.header.stamp = ros::Time::now();
    setpoint.command.header.stamp = ros::Time::now();

    // Assign position from EGO-Planner
    setpoint.state.pose.position.x = msg->position.x;
    setpoint.state.pose.position.y = msg->position.y;
    setpoint.state.pose.position.z = msg->position.z;

    // Convert yaw to quaternion
    setpoint.state.pose.orientation = normalizeQuaternion(msg->yaw);

    // Set velocity
    setpoint.state.velocity.linear.x = msg->velocity.x;
    setpoint.state.velocity.linear.y = msg->velocity.y;
    setpoint.state.velocity.linear.z = msg->velocity.z;

    // Set acceleration
    setpoint.state.acceleration.linear.x = msg->acceleration.x;
    setpoint.state.acceleration.linear.y = msg->acceleration.y;
    setpoint.state.acceleration.linear.z = msg->acceleration.z;

    // Append to trajectory message
    trajectory_msg.points.push_back(setpoint);


    // Set motor commands TODO: change from python to C++
    // setpoint.command.collective_thrust = 9.81 
    // setpoint.command.bodyrates = Vector3(0.0, 0.0, 0.0)  
    // setpoint.command.thrusts = [1.0, 1.0, 1.0, 1.0]


    // Publish the converted trajectory
    traj_pub.publish(trajectory_msg);
    // ROS_INFO("Published trajectory to Agilicious: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",
    //          msg->position.x, msg->position.y, msg->position.z, msg->yaw);
}









int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle nh;

    // Publisher for Agilicious trajectory
    traj_pub = nh.advertise<agiros_msgs::Reference>("/kingfisher/agiros_pilot/trajectory", 10);

    // Subscribe to EGO-Planner trajectory topic
    ros::Subscriber traj_sub = nh.subscribe("/planning/pos_cmd", 10, trajectoryCallback);


    ros::spin();
    return 0;
}
