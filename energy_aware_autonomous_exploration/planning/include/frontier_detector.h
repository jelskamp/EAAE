#ifndef FRONTIER_DETECTOR_H
#define FRONTIER_DETECTOR_H

#include <geometry_msgs/Point.h>
#include <octomap/OcTree.h>
#include <vector>

// #include "autonomous_exploration/ClusterInfo.h"
// #include "autonomous_exploration/ClusterInfoArray.h"



struct ClusterInfo {
    geometry_msgs::Point centroid;  // Centroid of the cluster
    int cluster_size;  // Number of points in the cluster
};


// Function prototypes
std::vector<ClusterInfo> divisiveKMeansClustering(std::vector<geometry_msgs::Point>& points);
geometry_msgs::Point computeCentroid(const std::vector<geometry_msgs::Point>& points);
bool isFrontier(std::shared_ptr<octomap::OcTree> octree, const octomap::point3d& node);
void publishFrontiers(const std::vector<ClusterInfo>& cluster_infos);
void publishFrontiers2(const std::vector<geometry_msgs::Point>& frontiers);




// Function prototypes
// autonomous_exploration::ClusterInfoArray divisiveKMeansClustering(std::vector<geometry_msgs::Point>& points);
// geometry_msgs::Point computeCentroid(const std::vector<geometry_msgs::Point>& points);
// bool isFrontier(std::shared_ptr<octomap::OcTree> octree, const octomap::point3d& node);
// void publishFrontiers(const autonomous_exploration::ClusterInfoArray& cluster_infos);
// void publishFrontiers2(const std::vector<geometry_msgs::Point>& frontiers);


#endif