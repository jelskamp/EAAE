#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <random>  
#include <agiros_msgs/QuadState.h>

#include <std_msgs/Bool.h>

// CHECK IF TRUE OR FALSE!!!
bool waypoint_reached = true;


// New struct to store cluster info
struct ClusterInfo {
    geometry_msgs::Point centroid;  // Centroid of the cluster
    int cluster_size;  // Number of points in the cluster
    // double energy_to_reach_cluster;
};


class FrontierDetector {
private:
    ros::NodeHandle nh;
    ros::Subscriber octomap_sub;
    ros::Publisher frontier_pub;
    ros::Publisher frontier_pub2;
    ros::Subscriber uav_pos;
    ros::Subscriber waypoint_reached_sub;
    // ros::Subscriber energy_sub;
    // ros::Publisher pot_target_pub
    double resolution;

    std::vector<ClusterInfo> last_valid_clusters;

    double uav_x = 0.0;
    double uav_y = 0.0;
    double uav_z = 0.0;

    // std::vector<ClusterInfo> candidate_clusters_;
    bool energy_received_ = false;
    // double latest_energy_ = 0.0;


    
public:
    FrontierDetector() {

        // CONT. SUBSCRIBER (WORKS)
        octomap_sub = nh.subscribe("/octomap_binary", 1, &FrontierDetector::octomapCallback, this);

        frontier_pub = nh.advertise<visualization_msgs::MarkerArray>("/frontier_markers", 1);
        frontier_pub2 = nh.advertise<visualization_msgs::MarkerArray>("/frontier_markers_all", 1);

        uav_pos = nh.subscribe("/kingfisher/agiros_pilot/state", 10, &FrontierDetector::uavPositionCallback, this);

        waypoint_reached_sub = nh.subscribe("/waypoint_reached", 10, &FrontierDetector::waypointReachedCallback, this);

        // energy_sub = nh.subscribe("/energy_consumed", 1, &FrontierDetector::energyCallback, this);
        // pot_target_pub = nh.advertise<geometry_msgs::PoseStamped>("/pot_target", 1); 

    }


    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
        std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg)));
        if (!octree) {
            ROS_ERROR("Failed to convert octomap message to OcTree.");
            return;
        }
        resolution = octree->getResolution();

        detectFrontiers(octree);
    }



    void detectFrontiers(std::shared_ptr<octomap::OcTree> octree) {
        std::vector<geometry_msgs::Point> frontiers;

        for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
            if (octree->isNodeOccupied(*it)) continue; // Ignore occupied nodes

            octomap::point3d node_coord = it.getCoordinate();

            if (node_coord.z() > 0.3 || node_coord.z() < 0.0) continue;  // Ignore points above 0.3m and below ground

            if (isFrontier(octree, node_coord)) {
                geometry_msgs::Point p;
                p.x = node_coord.x();
                p.y = node_coord.y();
                p.z = node_coord.z();
                frontiers.push_back(p);
            }
        }

        // Only this line in order to correctly, dynamically visualize frontiers (not clustered)
        publishFrontiers2(frontiers);

        std::vector<ClusterInfo> cluster_infos = divisiveKMeansClustering(frontiers);
        publishFrontiers(cluster_infos);

        findAndPublishBestWaypoint(cluster_infos);
    }




    bool isFrontier(std::shared_ptr<octomap::OcTree> octree, const octomap::point3d& node) {
        static const double neighbor_offsets[4][3] = {
            {resolution, 0, 0},   // Step +1 in X direction
            {-resolution, 0, 0},  // Step -1 in X direction
            {0, resolution, 0},   // Step +1 in Y direction
            {0, -resolution, 0}   // Step -1 in Y direction
        };

        bool has_unknown_neighbor = false;
        bool has_free_neighbor = false;

        for (int i = 0; i < 4; ++i) {  // Loop over only X and Y directions
            const auto& offset = neighbor_offsets[i];
            octomap::point3d neighbor(node.x() + offset[0], node.y() + offset[1], node.z());
            octomap::OcTreeNode* neighbor_node = octree->search(neighbor);

            if (!neighbor_node) {
                has_unknown_neighbor = true;  // This means there's an unknown space nearby
            } else if (octree->isNodeOccupied(neighbor_node)) {
                continue;  // Skip occupied nodes
            } else {
                has_free_neighbor = true;
            }
        }

        return has_unknown_neighbor && has_free_neighbor;  // Only return true if BOTH conditions are met
    }


    double getFarthestPointDistance(const std::vector<geometry_msgs::Point>& cluster) {
        geometry_msgs::Point centroid = computeCentroid(cluster);
        double max_distance = 0.0;
    
        for (const auto& p : cluster) {
            double distance = sqrt(pow(p.x - centroid.x, 2) + pow(p.y - centroid.y, 2) + pow(p.z - centroid.z, 2));
            if (distance > max_distance) {
                max_distance = distance;
            }
        }
        return max_distance;
    }
    




    void waypointReachedCallback(const std_msgs::Bool::ConstPtr& msg) {
        waypoint_reached = msg->data;
        ROS_WARN_STREAM("Waypoint reached status updated: " << waypoint_reached);
    }
    


    // REVISED FUNCTION USING ACTUAL DIVISIVE K-MEANS CLUSTERING
    std::vector<ClusterInfo> divisiveKMeansClustering(std::vector<geometry_msgs::Point>& points) {
        
        // POSSIBLY ADD WAITER TO CALCULATE CLUSTERPOINTS 1/2 SECONDS AFTER WAYPOINT REACHED???
        if (!waypoint_reached) {
            ROS_WARN("Skipping clustering - UAV is still moving.");
            return last_valid_clusters;  // Return empty cluster list until UAV reaches waypoint
        }
        ros::Duration(2.0).sleep();
        waypoint_reached = false;  // Reset waypoint status after recalculating clusters
    
    // std::vector<ClusterInfo> divisiveKMeansClustering(std::vector<geometry_msgs::Point>& points) {
        std::vector<ClusterInfo> cluster_infos;
        std::vector<std::vector<geometry_msgs::Point>> clusters;
        clusters.push_back(points);

        while (!clusters.empty()) {
            std::vector<geometry_msgs::Point> current_cluster = clusters.back();
            clusters.pop_back();

            // double cutoff_distance = 10;  // Stop splitting if the farthest point distance is below this threshold
            double cutoff_distance = (5) * tan(1.0 / 2.0);   // Hardcoded for now, LATER: use URDF param max_range and horizontal_fov
            if (getFarthestPointDistance(current_cluster) < cutoff_distance) {

                ROS_INFO_STREAM("\n\n Distnace less than cutoff: " << cutoff_distance << "\n\n");

                ClusterInfo cluster_info;
                cluster_info.centroid = computeCentroid(current_cluster);
                cluster_info.cluster_size = current_cluster.size();
                cluster_infos.push_back(cluster_info);
                continue;
            }

            std::vector<geometry_msgs::Point> cluster1, cluster2;

            // Ensure at least two points exist for random selection
            if (current_cluster.size() < 2) {
                
                ROS_INFO_STREAM("\n\n CLUSTER SIZE LESS THAN 2 \n\n");

                ClusterInfo cluster_info;
                cluster_info.centroid = computeCentroid(current_cluster);
                cluster_info.cluster_size = current_cluster.size();
                cluster_infos.push_back(cluster_info);
                continue;
            }

            // Randomly select two initial points as cluster centroids
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> distrib(0, current_cluster.size() - 1);

            geometry_msgs::Point centroid1 = current_cluster[distrib(gen)];
            geometry_msgs::Point centroid2 = current_cluster[distrib(gen)];

            // Assign each point to the closest centroid
            for (const auto& p : current_cluster) {
                double dist1 = sqrt(pow(p.x - centroid1.x, 2) + pow(p.y - centroid1.y, 2) + pow(p.z - centroid1.z, 2));
                double dist2 = sqrt(pow(p.x - centroid2.x, 2) + pow(p.y - centroid2.y, 2) + pow(p.z - centroid2.z, 2));

                if (dist1 < dist2) {
                    cluster1.push_back(p);
                } else {
                    cluster2.push_back(p);
                }
            }

            if (!cluster1.empty()) clusters.push_back(cluster1);
            if (!cluster2.empty()) clusters.push_back(cluster2);
        }

        last_valid_clusters = cluster_infos;
        return cluster_infos;
    }









    // Compute the centroid of a given cluster of points
    geometry_msgs::Point computeCentroid(const std::vector<geometry_msgs::Point>& points) {
        geometry_msgs::Point centroid;
        double sum_x = 0, sum_y = 0, sum_z = 0;
        for (const auto& p : points) {
            sum_x += p.x;
            sum_y += p.y;
            sum_z += p.z;
        }
        centroid.x = sum_x / points.size();
        centroid.y = sum_y / points.size();
        centroid.z = sum_z / points.size();
        return centroid;
    }



    
    
    void uavPositionCallback(const agiros_msgs::QuadState::ConstPtr &msg) {
        uav_x = msg->pose.position.x;
        uav_y = msg->pose.position.y;
        uav_z = msg->pose.position.z;
    }

    double calculateYaw(double x, double y) {
        
        double dx = x - uav_x;
        double dy = y - uav_y;

        double yaw;
        if (dx == 0) {
            if (dy > 0) {
                yaw = M_PI / 2;
            } else {
                yaw = -1 * M_PI / 2;
            }
        } else if (dx > 0) {
            yaw = atan2(dy, dx);
        } else {
            yaw = atan2(dy, dx) + M_PI;
        }
    
        ROS_INFO_STREAM("Calculated yaw: " << yaw);
        return yaw;
    }
    
    
    void findAndPublishBestWaypoint(const std::vector<ClusterInfo>& cluster_infos) {
        // Check if there are any clusters
        if (cluster_infos.empty()) {
            ROS_WARN("No clusters found. No waypoint published.");
            return;
        }


        // Find the largest cluster by size > LATER: taking distance into account
        auto largest_cluster = std::max_element(cluster_infos.begin(), cluster_infos.end(), 
            [](const ClusterInfo& a, const ClusterInfo& b) {
                return a.cluster_size < b.cluster_size;
            }
        );



    
        // Extract the centroid as the target position
        double target_x = largest_cluster->centroid.x;
        double target_y = largest_cluster->centroid.y;
        double target_z = largest_cluster->centroid.z;
        
    
        // Calculate yaw based on target THIS IS WRONG, SHOULD USE UAV POSITION
        // double target_yaw = atan2(target_y, target_x);
        // ATTEMPT with function: (I believe it works)
        double target_yaw = calculateYaw(target_x, target_y);

    
        // Publish the best waypoint
        geometry_msgs::PoseStamped waypoint;
        waypoint.header.stamp = ros::Time::now();
        waypoint.header.frame_id = "world";
        
        waypoint.pose.position.x = target_x;
        waypoint.pose.position.y = target_y;
        waypoint.pose.position.z = target_z;
    
        // Calculate quaternion for yaw rotation
        waypoint.pose.orientation.x = 0.0;
        waypoint.pose.orientation.y = 0.0;
        waypoint.pose.orientation.z = sin(target_yaw / 2);
        waypoint.pose.orientation.w = cos(target_yaw / 2);
    
        // Use a new publisher for best waypoint
        static ros::NodeHandle nh;
        static ros::Publisher best_waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/best_waypoint", 10);
    
        ROS_INFO_STREAM("Publishing best waypoint: x=" << target_x << ", y=" << target_y << ", z=" << target_z << ", yaw=" << target_yaw);
        best_waypoint_pub.publish(waypoint);
        
    }


  



    // void publishPotentialTarget(const std::vector<geometry_msgs::Point>& pot_target_centroid) {

    //     geometry_msgs::PoseStamped pot_target;
    //     pot_target.header.stamp = ros::Time::now();
    //     pot_target.header.frame_id = "world";
        
    //     pot_target.pose.position.x = pot_target_centroid.x;
    //     pot_target.pose.position.y = pot_target_centroid.y;
    //     pot_target.pose.position.z = pot_target_centroid.z;
    
    //     // Calculate quaternion for yaw rotation
    //     pot_target.pose.orientation.x = 0.0;
    //     pot_target.pose.orientation.y = 0.0;
    //     pot_target.pose.orientation.z = sin(0);  //TODO!!! THINK OF
    //     pot_target.pose.orientation.w = cos(0);  //TODO!!! THINK OF
    
    //     // Use a new publisher for best waypoint
    //     // static ros::NodeHandle nh;
    //     // static ros::Publisher best_waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/pot_target", 10);
    
    //     ROS_INFO_STREAM("Publishing pot target...");
    //     best_waypoint_pub.publish(pot_target);

    // }




    void publishFrontiers(const std::vector<ClusterInfo>& cluster_infos) {
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        
        ROS_INFO_STREAM("Publishing frontier markers with frame_id: " << marker.header.frame_id);
        
        marker.ns = "frontiers";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = marker.scale.y = marker.scale.z = resolution;
        // marker.color.a = 1.0;
        // marker.color.r = 1.0;
        // marker.color.g = 1.0;
        // marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;



        // Loop through each cluster and add its centroid as a marker
        for (const auto& cluster_info : cluster_infos) {
            // Extract the centroid
            geometry_msgs::Point point = cluster_info.centroid;
            marker.points.push_back(point);

        }
        marker_array.markers.push_back(marker);
        // Publish the markers to RViz
        frontier_pub.publish(marker_array);

    }

    
    void publishFrontiers2(const std::vector<geometry_msgs::Point>& frontiers) {
        visualization_msgs::MarkerArray marker_array2;
        visualization_msgs::Marker marker2;
        marker2.header.frame_id = "world";
        marker2.header.stamp = ros::Time::now();


        
        ROS_INFO_STREAM("Publishing frontier markers with frame_id: " << marker2.header.frame_id);
        
        marker2.ns = "frontiers2";
        marker2.id = 1;
        marker2.type = visualization_msgs::Marker::SPHERE_LIST;
        marker2.action = visualization_msgs::Marker::ADD;
        marker2.scale.x = marker2.scale.y = marker2.scale.z = resolution;
        // marker2.color.a = 1.0;
        // marker2.color.r = 1.0;
        // marker2.color.g = 0.0;
        // marker2.color.b = 1.0;
        marker2.color.a = 0.5;
        marker2.color.r = 1.0;
        marker2.color.g = 1.0;
        marker2.color.b = 0.0;



        for (const auto& point : frontiers) {
            marker2.points.push_back(point);
        }
        marker_array2.markers.push_back(marker2);
        frontier_pub2.publish(marker_array2);
    }


};





int main(int argc, char** argv) {
    ros::init(argc, argv, "frontier_detector");
    FrontierDetector fd;
    ros::spin();
    return 0;
}
