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
#include <std_msgs/Float64.h>
// includes for .srv files 
#include <project_msgs/getTrajectory.h>
#include <project_msgs/getEnergy.h>


struct ClusterInfo {
    geometry_msgs::Point centroid;  // Centroid of the cluster
    int cluster_size;  // Number of points in the cluster
    agiros_msgs::Reference trajectory_to_cluster;
    double energy_to_reach_cluster;
};


class FrontierDetector {
private:
    ros::NodeHandle nh;
    // Subscribers
    ros::Subscriber octomap_sub;
    ros::Subscriber uav_pos_sub;
    ros::Subscriber waypoint_reached_sub;
    ros::Subscriber energy_sub;
    // Publishers
    ros::Publisher cluster_pub;
    ros::Publisher frontier_pub;
    ros::Publisher pot_target_pub;
    ros::Publisher best_waypoint_pub;
    // Service clients
    ros::ServiceClient traj_client;
    ros::ServiceClient energy_client;


    std::vector<ClusterInfo> last_valid_clusters;
    std::vector<ClusterInfo> candidate_clusters_;
    
    double resolution;

    double uav_x = 0.0;
    double uav_y = 0.0;
    double uav_z = 0.0;

    bool exploration_ready_ = true;
    bool energy_received_ = false;
    double latest_energy_ = 0.0;

    
public:
    FrontierDetector() {
        octomap_sub = nh.subscribe("/octomap_binary", 1, &FrontierDetector::octomapCallback, this);

        cluster_pub = nh.advertise<visualization_msgs::MarkerArray>("/cluster_markers", 1, true); //prev: frontier_markers
        frontier_pub = nh.advertise<visualization_msgs::MarkerArray>("/frontier_markers", 1, true); //prev: frontier_markers_all

        uav_pos_sub = nh.subscribe("/kingfisher/agiros_pilot/state", 10, &FrontierDetector::uavPositionCallback, this);

        waypoint_reached_sub = nh.subscribe("/waypoint_reached", 10, &FrontierDetector::waypointReachedCallback, this);
        
        pot_target_pub = nh.advertise<geometry_msgs::PoseStamped>("/pot_target", 1); 

        best_waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/best_waypoint", 10, true);
    }


    // callback to get octomap and start detectFrontiers()
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
        std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg)));
        if (!octree) {
            ROS_ERROR("Failed to convert octomap message to OcTree.");
            return;
        }
        resolution = octree->getResolution();

        detectFrontiers(octree);
    }

    // detect frontiers and start exploration cycle (this happens contiously, also during flight to target)
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
        publishFrontiers(frontiers);


        //Add logic such that exploration cycle only starts once exploration_ready_ = true
        if (exploration_ready_) {
            exploration_ready_ = false;

            ROS_INFO_STREAM("start1 STARTING clustering...................1...............");
            // std::vector<ClusterInfo> cluster_infos = divisiveKMeansClustering(frontiers);
            std::vector<ClusterInfo> cluster_infos = divisiveKMeansClustering(frontiers, octree);


            ROS_INFO_STREAM("start2 STARTING publishing cluster infos.............2.............");
            publishClusters(cluster_infos);



            // uncomment for EAAE 
            ROS_INFO_STREAM("start3 STARTING getTrajectory................3.................");
            std::vector<ClusterInfo> candidate_clusters_ = getTrajectoryForCandidateClusters(cluster_infos);

            ROS_INFO_STREAM("start4 STARTING getEnergy................4.................");
            candidate_clusters_ = getEnergyForCandidateClusters(candidate_clusters_);
        
            ROS_INFO_STREAM("start5 STARTING selectBest................5.................");
            ClusterInfo target_cluster = selectBestCluster(candidate_clusters_);




            // NON-ENERGY-AWARE:
            // ClusterInfo target_cluster = selectBestCluster2(cluster_infos);




            ROS_INFO_STREAM("start6 STARTING publishFinalTarget................6.................");
            publishFinalTarget(target_cluster.centroid);

        } else {
            ROS_INFO_STREAM("!!! ------- Exploration paused. UAV is en route to final target or still processing. -------- !!!");
        }


        // prev. logic where waypoint_reached/exploration_ready bool was used within functions....
        // ROS_INFO_STREAM("1 STARTING DIV KMEANS CLUSTERING...................1...............");
        // std::vector<ClusterInfo> cluster_infos = divisiveKMeansClustering(frontiers);
        // ROS_INFO_STREAM("2 STARTING PUBLISHING FRONTIER CLUSTER CENTROIDS.............2.............");
        // publishClusters(cluster_infos);
        // ROS_INFO_STREAM("3 STARTING FINDandPUBLISH BEST WAYPOINT................3.................");
        // std::vector<ClusterInfo> candidate_clusters_ = getTrajectoryForCandidateClusters(cluster_infos);
        // candidate_clusters_ = getEnergyForCandidateClusters(candidate_clusters_);
        // ClusterInfo target_cluster = selectBestCluster(candidate_clusters_);
        // publishFinalTarget(target_cluster.centroid);
    }


    //utlility: check if voxel is frontier
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

    //utility: get farthest point from cluster centoid
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


    // callback for when the waypoint is reached and a new exploration cycle should start
    void waypointReachedCallback(const std_msgs::Bool::ConstPtr& msg) {

        if (msg->data) {
            ROS_INFO("Waypoint REACHED. READY for next EXPLORATION CYCLE.");
            exploration_ready_ = true;
        }
    }

    //utility: compute the centroid of cluster 
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

    // callback for uav position
    void uavPositionCallback(const agiros_msgs::QuadState::ConstPtr &msg) {
        uav_x = msg->pose.position.x;
        uav_y = msg->pose.position.y;
        uav_z = msg->pose.position.z;
    }

    // calculate heading from current uav position to target
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
    

    double calculateDistance(const geometry_msgs::Point& p1, double x, double y, double z) {

        double dist_x = uav_x - p1.x;
        double dist_y = uav_y - p1.y;
        double dist_z = uav_z - p1.z;

        double distance = sqrt(pow(dist_x, 2) + pow(dist_y, 2) + pow(dist_z, 2));

        
        return distance;
    }


    bool isSurroundingFree(const geometry_msgs::Point& center, std::shared_ptr<octomap::OcTree> octree) {
        double resolution = octree->getResolution();
        octomap::point3d origin(center.x, center.y, center.z);

        // Loop through a 7x7x3 region around the center voxel (centred at (4,4,1))
        for (int dx = -5; dx <= 5; ++dx) {
            for (int dy = -5; dy <= 5; ++dy) {
                for (int dz = 5; dz <= 8; ++dz) {  // only above the centroid
                    octomap::point3d check_point = origin;
                    check_point.x() += dx * resolution;
                    check_point.y() += dy * resolution;
                    check_point.z() += dz * resolution;

                    auto node = octree->search(check_point);
                    if (node && octree->isNodeOccupied(node)) {
                        return false;  // found occupied voxel
                    }
                }
            }
        }
        return true;  // all surrounding voxels are free
    }




    // clustering
    // std::vector<ClusterInfo> divisiveKMeansClustering(std::vector<geometry_msgs::Point>& points) {
    
    std::vector<ClusterInfo> divisiveKMeansClustering(std::vector<geometry_msgs::Point>& points, std::shared_ptr<octomap::OcTree> octree) {

        
        // Possibly delete this sleeper?
        // ros::Duration(2.0).sleep();
    
        std::vector<ClusterInfo> cluster_infos;
        std::vector<std::vector<geometry_msgs::Point>> clusters;
        clusters.push_back(points);

        while (!clusters.empty()) {
            std::vector<geometry_msgs::Point> current_cluster = clusters.back();
            clusters.pop_back();

            // Stop splitting if the farthest point distance is below this threshold
            double cutoff_distance = (5) * tan(1.0 / 2.0);   // Hor FoV = 1.0rad , (Hardcoded for now, LATER: use URDF param max_range and horizontal_fov?
            if (getFarthestPointDistance(current_cluster) < cutoff_distance) {

                ROS_INFO_STREAM("\n\n Cluster found: cluster diameter less than cutoff: " << cutoff_distance << "\n");

                ClusterInfo cluster_info;
                cluster_info.centroid = computeCentroid(current_cluster);
                cluster_info.cluster_size = current_cluster.size();

                // ADD FLITER FOR FEASIBILTY PLANNER
                if (!isSurroundingFree(cluster_info.centroid, octree)) {
                    ROS_WARN_STREAM("--------------");
                    ROS_INFO_STREAM("CLUSTER CENTROID NOT FREE SURROUDING");
                    ROS_INFO_STREAM("centroid: " << cluster_info.centroid.x << " " << cluster_info.centroid.y << " " << cluster_info.centroid.z);
                    ROS_WARN_STREAM("--------------");
                    continue;
                }

                cluster_infos.push_back(cluster_info);

                ROS_INFO_STREAM("Current cluster centroid: " << cluster_info.centroid.x << " " << cluster_info.centroid.y << " " << cluster_info.centroid.z);
                ROS_INFO_STREAM("Current cluster size: " << cluster_info.cluster_size);

                continue;
            }

            std::vector<geometry_msgs::Point> cluster1, cluster2;

            // Ensure at least two points exist for random selection
            if (current_cluster.size() < 2) {
                
                ROS_INFO_STREAM("\n\n CLUSTER SIZE LESS THAN 2 \n\n");

                ClusterInfo cluster_info;
                cluster_info.centroid = computeCentroid(current_cluster);
                cluster_info.cluster_size = current_cluster.size();

                // ADD FLITER FOR FEASIBILTY PLANNER
                if (!isSurroundingFree(cluster_info.centroid, octree)) {
                    ROS_WARN_STREAM("--------------");
                    ROS_INFO_STREAM("CLUSTER CENTROID NOT FREE SURROUDING");
                    ROS_INFO_STREAM("centroid: " << cluster_info.centroid.x << " " << cluster_info.centroid.y << " " << cluster_info.centroid.z);
                    ROS_WARN_STREAM("--------------");
                    continue;
                }

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


        // IMPLEMENT EXTRA CANDIDATE CLUSTER CHECKS FOR PLANNER FEASIBILITY



        last_valid_clusters = cluster_infos;
        return cluster_infos;
    }

    // getting trajectories for largest clusters
    std::vector<ClusterInfo> getTrajectoryForCandidateClusters(const std::vector<ClusterInfo>& cluster_infos) {
        // CHECK!! not 100% sure about this line yet (almost sure)
        candidate_clusters_.clear();

        // Check if there are any clusters
        if (cluster_infos.empty()) {
            ROS_WARN("No clusters found. No waypoint published.");
            // TODO: what to about this check???!! what to return?
            // return;
        }
  

        
        int num_candidates = 3;     //largest 3? why?
        // Copy cluster_infos so we can sort it
        std::vector<ClusterInfo> sorted_clusters = cluster_infos;

        // Sort clusters descending by size
        std::sort(sorted_clusters.begin(), sorted_clusters.end(), 
            [](const ClusterInfo& a, const ClusterInfo& b) {
                return a.cluster_size > b.cluster_size;  // Larger clusters first
            }
        );



        // Filter clusters: only keep those at least 2m away from UAV
        std::vector<ClusterInfo> valid_clusters;
        for (const auto& cluster : sorted_clusters) {
            double distance = calculateDistance(cluster.centroid, uav_x, uav_y, uav_z);
            ROS_INFO_STREAM("DISTANCE TO CLUSTER: >>> " << distance);
            if (distance >= 2.9) {
                valid_clusters.push_back(cluster);
            }
        }

        if (valid_clusters.empty()) {
            ROS_ERROR("---- All clusters are too close (<2m). No valid candidates. ----");

            // return {};  // Return empty vector or handle accordingly
        }



        // Select top N (or all if fewer)
        std::vector<ClusterInfo> top_clusters;
        for (int i = 0; i < std::min(num_candidates, static_cast<int>(valid_clusters.size())); ++i) {
            top_clusters.push_back(valid_clusters[i]);
        }


        // Logic for trajectory service client request:
        traj_client = nh.serviceClient<project_msgs::getTrajectory>("get_trajectory");

        // Wait for the client-server connection to be ready
        ROS_INFO("--- Waiting for 'get_trajectory' service...");
        if (!traj_client.waitForExistence(ros::Duration(10.0))) {
            ROS_ERROR("Service 'get_trajectory' not available. Aborting...");
            // return candidate_clusters_;  // or handle differently
        }

        for (auto& cluster : top_clusters) {
     
            project_msgs::getTrajectory srv;

            double target_yaw = calculateYaw(cluster.centroid.x, cluster.centroid.y);
            geometry_msgs::PoseStamped pot_target_input;
            pot_target_input.header.stamp = ros::Time::now();
            pot_target_input.header.frame_id = "world";
            pot_target_input.pose.position = cluster.centroid;
            pot_target_input.pose.position.z = 1.0;
            pot_target_input.pose.orientation.w = 1.0;

            pot_target_input.pose.orientation.x = 0.0;
            pot_target_input.pose.orientation.y = 0.0;
            pot_target_input.pose.orientation.z = sin(target_yaw / 2);
            pot_target_input.pose.orientation.w = cos(target_yaw / 2);


            srv.request.pot_target = pot_target_input;            


            // BLOCKING LOOP: keep calling until service responds
            bool success_traj = false;
            while (ros::ok() && !success_traj) {
                ROS_INFO("Calling trajectory service...");
                success_traj = traj_client.call(srv);
                ROS_INFO_STREAM("Call succesful: " << success_traj);
                
                // ROS_INFO_STREAM(srv.request.pot_target);
                // ROS_INFO_STREAM(srv.response.global_trajectory);

                if (!success_traj) {
                    ROS_WARN("Call to TRAJECTORY service FAILED. Retrying in 0.5s...");
                    ros::Duration(0.5).sleep();
                }
            }


            cluster.trajectory_to_cluster = srv.response.global_trajectory;
            candidate_clusters_.push_back(cluster);
            ROS_INFO("---- TRAJECTORY ADDED:! ----");

        }

        ROS_WARN_STREAM("-----------------");
        ROS_WARN_STREAM("--- OUT OF Traj Loop ! ----");
        ROS_WARN_STREAM("-----------------");
        
        return candidate_clusters_;
    }

    // getting energy for trajectories to largest clusters
    std::vector<ClusterInfo> getEnergyForCandidateClusters(std::vector<ClusterInfo>& candidate_clusters_){
        // TODO: EDGE CASE CHECKS HERE:


        // DEBUG statement to check if we have received distinct trajs.
        ROS_INFO_STREAM("x,y,z for cluster 1:  " << candidate_clusters_[0].centroid.x << " , " << candidate_clusters_[0].centroid.y << " , " << candidate_clusters_[0].centroid.z);
        // ROS_INFO_STREAM(candidate_clusters_[0].trajectory_to_cluster.points.size());
        ROS_INFO_STREAM("x,y,z for cluster 2:  " << candidate_clusters_[1].centroid.x << " , " << candidate_clusters_[1].centroid.y << " , " << candidate_clusters_[1].centroid.z);
        // ROS_INFO_STREAM(candidate_clusters_[1].trajectory_to_cluster.points.size());
        ROS_INFO_STREAM("x,y,z for cluster 3:  " << candidate_clusters_[2].centroid.x << " , " << candidate_clusters_[2].centroid.y << " , " <<  candidate_clusters_[2].centroid.z);
        // ROS_INFO_STREAM(candidate_clusters_[2].trajectory_to_cluster.points.size());


        // Logic for trajectory service client request:
        energy_client = nh.serviceClient<project_msgs::getEnergy>("/energy_estimator_service/get_energy");


        // Wait for the client-server connection to be ready
        ROS_INFO("--- Waiting for 'get_energy' service...");
        if (!energy_client.waitForExistence(ros::Duration(10.0))) {
            ROS_ERROR("Service 'get_energy' not available. Aborting...");
            // TODO: how to handle? 
            // return candidate_clusters_;  
        }


        for (auto& cluster : candidate_clusters_) {
     
            project_msgs::getEnergy srv;
            srv.request.pot_trajectory = cluster.trajectory_to_cluster;


            // BLOCKING LOOP: keep calling until service responds
            bool success_energy = false;
            while (ros::ok() && !success_energy) {
                ROS_INFO("Calling energy service...");
                success_energy = energy_client.call(srv);

                // ROS_INFO_STREAM(cluster.trajectory_to_cluster);

                if (!success_energy) {
                    ROS_WARN("Call to ENERGY service FAILED. Retrying in 0.5s...");
                    ros::Duration(0.5).sleep();
                }
            }

            cluster.energy_to_reach_cluster = srv.response.energy;
            // COMMENTING THIS LINE BELOW RESOLVED 'std::length_error' ISSUE... SHOULD BE REDUNDANT BC. "CLUSTER in CLUSTERS:"
            // candidate_clusters_.push_back(cluster);
            ROS_INFO("---- ENERGY ADDED! ----");

        }

        ROS_WARN_STREAM("-----------------");
        ROS_WARN_STREAM("--- OUT OF Energy Loop ! ----");
        ROS_WARN_STREAM("-----------------");

        return candidate_clusters_;
    }

    // selecting final target   > TODO: decison logic
    ClusterInfo selectBestCluster(const std::vector<ClusterInfo>& candidates) {
        if (candidates.empty()) {
            ROS_WARN("No candidate clusters provided. Returning default cluster.");
            return ClusterInfo();  // Default-constructed cluster   . TODO!
        }
    
        // Placeholder: just return the first one for now

        // ROS_INFO_STREAM("SELECTED first CLUSTER with energy: " << candidates[0].energy_to_reach_cluster);

        ClusterInfo best_cluster = candidates[0];

        for (const auto& cluster : candidates) {
            if (cluster.energy_to_reach_cluster < best_cluster.energy_to_reach_cluster) {
                best_cluster = cluster;
            }
        }

        ROS_INFO_STREAM("SELECTED cluster with LOWEST energy: " << best_cluster.energy_to_reach_cluster);

        // return candidates[0];
        return best_cluster;
    }
    


    // selecting final target: for NON-ENERGY-AWARE:
    ClusterInfo selectBestCluster2(const std::vector<ClusterInfo>& cluster_infos) {
        candidate_clusters_.clear();

        if (cluster_infos.empty()) {
            ROS_WARN("No candidate clusters provided. Returning default cluster.");
            return ClusterInfo();  // Default-constructed cluster   . TODO!
        }
  

        
        int num_candidates = 3;     //largest 3? why?
        // Copy cluster_infos so we can sort it
        std::vector<ClusterInfo> sorted_clusters = cluster_infos;

        // Sort clusters descending by size
        std::sort(sorted_clusters.begin(), sorted_clusters.end(), 
            [](const ClusterInfo& a, const ClusterInfo& b) {
                return a.cluster_size > b.cluster_size;  // Larger clusters first
            }
        );



        // Filter clusters: only keep those at least 2m away from UAV       > CAN BE DELETED
        std::vector<ClusterInfo> valid_clusters;
        for (const auto& cluster : sorted_clusters) {
            double distance = calculateDistance(cluster.centroid, uav_x, uav_y, uav_z);
            ROS_INFO_STREAM("DISTANCE TO CLUSTER: >>> " << distance);
            if (distance >= 2.1) {
                valid_clusters.push_back(cluster);
            }
        }

        if (valid_clusters.empty()) {
            ROS_ERROR("---- All clusters are too close (<2.1m). No valid candidates. ----");

            // return {};  // Return empty vector or handle accordingly
        }





        // return biggest cluster:

        return valid_clusters[0];
    }


    // publishing 

    void publishFrontiers(const std::vector<geometry_msgs::Point>& frontiers) {
        visualization_msgs::MarkerArray marker_array2;
        visualization_msgs::Marker marker2;
        marker2.header.frame_id = "world";
        marker2.header.stamp = ros::Time::now();


        
        // ROS_INFO_STREAM("Publishing frontier markers with frame_id: " << marker2.header.frame_id);
        
        marker2.ns = "frontiers";
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
        frontier_pub.publish(marker_array2);
    }

    void publishClusters(const std::vector<ClusterInfo>& cluster_infos) {
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        
        marker.ns = "clusters";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = marker.scale.y = marker.scale.z = resolution;

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
        cluster_pub.publish(marker_array);
    }

    void publishFinalTarget(const geometry_msgs::Point& final_target_centroid) {

        double target_x = final_target_centroid.x;
        double target_y = final_target_centroid.y;
        double target_z = final_target_centroid.z;
        
    
        // Calculate yaw based on target THIS IS WRONG, SHOULD USE UAV POSITION
        // double target_yaw = atan2(target_y, target_x);
        // ATTEMPT with function: (I believe it works)
        double target_yaw = calculateYaw(target_x, target_y);

    
        // Set msgs data
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
    
        // Publish best waypoint
        ROS_INFO_STREAM("Publishing FINAL TARGET: x=" << waypoint.pose.position.x << ", y=" << target_y << ", z=" << target_z << ", yaw=" << target_yaw);
        best_waypoint_pub.publish(waypoint);
    }


};





int main(int argc, char** argv) {
    ros::init(argc, argv, "frontier_detector");
    FrontierDetector fd;
    ros::spin();
    return 0;
}
