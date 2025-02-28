#ifndef MAPPING_NODE_H
#define MAPPING_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class MappingNode {
private:
    ros::NodeHandle nh;
    ros::Subscriber pointcloud_sub;
    ros::Publisher octomap_pub;
    octomap::OcTree *octree;

public:
    MappingNode();
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);
    ~MappingNode();
};

#endif
