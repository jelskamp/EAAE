#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_publisher_for_ego");
    ros::NodeHandle nh;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/my_odom_topic", 10);
    tf::TransformListener listener;

    ros::Rate rate(30); // 30 Hz

    while (ros::ok()) {
        tf::StampedTransform transform;
        try {
            // Listen for transform from world to UAV
            listener.lookupTransform("/world", "/kingfisher/base_link", ros::Time(0), transform);

            // Create Odometry message
            nav_msgs::Odometry odom_msg;
            odom_msg.header.stamp = ros::Time::now();
            odom_msg.header.frame_id = "/world";
            odom_msg.child_frame_id = "/kingfisher/base_link";

            // Set position
            odom_msg.pose.pose.position.x = transform.getOrigin().x();
            odom_msg.pose.pose.position.y = transform.getOrigin().y();
            odom_msg.pose.pose.position.z = transform.getOrigin().z();

            // Set orientation
            odom_msg.pose.pose.orientation.x = transform.getRotation().x();
            odom_msg.pose.pose.orientation.y = transform.getRotation().y();
            odom_msg.pose.pose.orientation.z = transform.getRotation().z();
            odom_msg.pose.pose.orientation.w = transform.getRotation().w();

            // Publish the message
            odom_pub.publish(odom_msg);

            // ROS_INFO("Published Odometry: x=%.2f, y=%.2f, z=%.2f",
            //          odom_msg.pose.pose.position.x,
            //          odom_msg.pose.pose.position.y,
            //          odom_msg.pose.pose.position.z);
        } catch (tf::TransformException &ex) {
            ROS_WARN("TF lookup failed: %s", ex.what());
        }

        rate.sleep();
    }

    return 0;
}
