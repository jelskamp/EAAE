#ifndef CUSTOM_OCTOMAP_SERVER_H_
#define CUSTOM_OCTOMAP_SERVER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <string>
#include <octomap_msgs/conversions.h>

class CustomOctomapServer {
public:
    CustomOctomapServer();
    ~CustomOctomapServer();

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    void processPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void publishOctomap();

private:
    ros::NodeHandle nh_;
    ros::Subscriber pointCloudSub_;
    ros::Publisher octomapBinaryPub_, occupiedCellsPub_, freeCellsPub_;

    tf::TransformListener tfListener_;
    octomap::OcTree* octree_;
    double maxRange_;

    std::string frameId_, baseFrameId_;
};

#endif
