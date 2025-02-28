#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <agiros_msgs/QuadState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>




void poseCallback(const agiros_msgs::QuadState::ConstPtr &msg) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "kingfisher/base_link"));
}   


int main(int argc, char** argv){
    ros::init(argc, argv, "uav_tf_broadcast");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/kingfisher/agiros_pilot/state", 10, &poseCallback);
    ros::spin();
    return 0;
}
