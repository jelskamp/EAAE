#include "perception/mapping_node.h"  
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

MappingNode::MappingNode() {
    // Subscribe to depth camera or LiDAR topic (Gazebo usually publishes to this)
    // pointcloud_sub = nh.subscribe("/camera/depth/points", 1, &MappingNode::pointCloudCallback, this);
    pointcloud_sub = nh.subscribe("/kingfisher/camera_depth/depth/points", 1, &MappingNode::pointCloudCallback, this);


    // Publish the generated OctoMap
    octomap_pub = nh.advertise<octomap_msgs::Octomap>("/octomap_binary", 1);

    // Initialize OctoMap with 0.1m resolution (adjust as needed)
    // TODO: make res variable to resolve double declaration in mapping_node.cpp and mapping.launch? or see where to remove.... !
    octree = new octomap::OcTree(0.1);
}

void MappingNode::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
    // Convert ROS PointCloud2 message to PCL PointCloud format
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    
    // Insert points into OctoMap
    for (auto &point : cloud.points) {
        octree->updateNode(octomap::point3d(point.x, point.y, point.z), true); // Mark as occupied
    }

    // Publish the updated OctoMap
    octomap_msgs::Octomap octomap_msg;
    octomap_msgs::binaryMapToMsg(*octree, octomap_msg);
    octomap_msg.header.frame_id = "world";          // Adjust to map for iris uav
    octomap_pub.publish(octomap_msg);

    ROS_INFO("OctoMap updated with %ld points", cloud.points.size());
}

MappingNode::~MappingNode() {
    delete octree;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mapping_node");
    MappingNode mapping_node;
    ros::spin();
    return 0;
}
