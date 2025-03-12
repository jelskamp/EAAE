#include "custom_octomap_server.h"

CustomOctomapServer::CustomOctomapServer() : nh_("~"), maxRange_(5.0) {
    nh_.param("frame_id", frameId_, std::string("world"));
    nh_.param("base_frame_id", baseFrameId_, std::string("/kingfisher/base_link"));
    nh_.param("sensor_model/max_range", maxRange_, 5.0);

    pointCloudSub_ = nh_.subscribe("/kingfisher/camera_depth/depth/points", 1, 
                                   &CustomOctomapServer::pointCloudCallback, this);

    octomapBinaryPub_ = nh_.advertise<octomap_msgs::Octomap>("/octomap_binary", 1);
    occupiedCellsPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/occupied_cells_vis_array", 1);
    freeCellsPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/free_cells_vis_array", 1);

    octree_ = new octomap::OcTree(0.1); // Resolution 0.1 (same as your launch file)
    ROS_INFO("Custom Octomap Server Initialized");
}

CustomOctomapServer::~CustomOctomapServer() {
    delete octree_;
}

void CustomOctomapServer::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    processPointCloud(cloud);
}


void CustomOctomapServer::processPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    tf::StampedTransform transform;
    try {
        tfListener_.lookupTransform(frameId_, cloud->header.frame_id, ros::Time(0), transform);
    } catch (tf::TransformException& ex) {
        ROS_WARN("Transform error: %s", ex.what());
        return;
    }

    octomap::point3d sensorOrigin(transform.getOrigin().x(),
                                  transform.getOrigin().y(),
                                  transform.getOrigin().z());

    for (const auto& point : cloud->points) {
        // Transform point
        tf::Vector3 tfPoint(point.x, point.y, point.z);
        tfPoint = transform * tfPoint;
        octomap::point3d endPoint(tfPoint.x(), tfPoint.y(), tfPoint.z());

        // Skip NaN points
        if (std::isnan(endPoint.x()) || std::isnan(endPoint.y()) || std::isnan(endPoint.z())) {
            continue;
        }

        // Compute direction vector safely
        octomap::point3d direction = endPoint - sensorOrigin;
        if (direction.norm() < 1e-6) {  
            continue;
        }
        direction.normalize();

        // Ensure point is in bounds before casting ray
        octomap::OcTreeKey key;
        if (!octree_->coordToKeyChecked(endPoint, key)) {
            ROS_WARN("Point out of octree bounds: %f, %f, %f", endPoint.x(), endPoint.y(), endPoint.z());
            continue;
        }

        // Cast ray
        octomap::point3d end;
        bool hit = octree_->castRay(sensorOrigin, direction, end, true, maxRange_);

        if (!hit) {
            octomap::KeyRay keyRay;
            if (octree_->computeRayKeys(sensorOrigin, endPoint, keyRay)) {
                for (const auto& key : keyRay) {
                    octree_->updateNode(key, false); // Mark as free
                }
            }
        } else {
            octree_->updateNode(end, true); // Mark as occupied
        }
    }

    publishOctomap();
}







// void CustomOctomapServer::publishOctomap() {
//     octomap_msgs::Octomap msg;
//     msg.header.frame_id = frameId_;
//     msg.header.stamp = ros::Time::now();

//     // if (octomap_msgs::fullMapToMsg(*octree_, msg)) {
//     if (octomap_msgs::binaryMapToMsg(*octree_, msg)) {

//         octomapBinaryPub_.publish(msg);
//     } else {
//         ROS_ERROR("Error serializing OctoMap");
//     }

//     visualization_msgs::MarkerArray occupiedNodes, freeNodes;
//     occupiedNodes.markers.resize(octree_->getTreeDepth() + 1);
//     freeNodes.markers.resize(octree_->getTreeDepth() + 1);

//     for (octomap::OcTree::iterator it = octree_->begin(octree_->getTreeDepth()), end = octree_->end(); it != end; ++it) {
//         double x = it.getX(), y = it.getY(), z = it.getZ();
//         geometry_msgs::Point cubeCenter;
//         cubeCenter.x = x;
//         cubeCenter.y = y;
//         cubeCenter.z = z;

//         if (octree_->isNodeOccupied(*it)) {
//             occupiedNodes.markers[it.getDepth()].points.push_back(cubeCenter);
//         } else {
//             freeNodes.markers[it.getDepth()].points.push_back(cubeCenter);
//         }
//     }

//     occupiedCellsPub_.publish(occupiedNodes);
//     freeCellsPub_.publish(freeNodes);
// }

void CustomOctomapServer::publishOctomap() {
    octomap_msgs::Octomap msg;
    msg.header.frame_id = frameId_;
    msg.header.stamp = ros::Time::now();

    if (octomap_msgs::binaryMapToMsg(*octree_, msg)) {
        octomapBinaryPub_.publish(msg);
    } else {
        ROS_ERROR("Error serializing OctoMap");
    }

    visualization_msgs::MarkerArray occupiedNodes, freeNodes;
    occupiedNodes.markers.resize(octree_->getTreeDepth() + 1);
    freeNodes.markers.resize(octree_->getTreeDepth() + 1);

    for (octomap::OcTree::iterator it = octree_->begin(octree_->getTreeDepth()), end = octree_->end(); it != end; ++it) {
        double x = it.getX(), y = it.getY(), z = it.getZ();
        geometry_msgs::Point cubeCenter;
        cubeCenter.x = x;
        cubeCenter.y = y;
        cubeCenter.z = z;

        unsigned depth = it.getDepth();
        if (octree_->isNodeOccupied(*it)) {
            occupiedNodes.markers[depth].points.push_back(cubeCenter);
        } else {
            freeNodes.markers[depth].points.push_back(cubeCenter);
        }
    }

    // 🛠 Ensure each marker array has the correct frame_id and properties
    for (size_t i = 0; i < occupiedNodes.markers.size(); ++i) {
        occupiedNodes.markers[i].header.frame_id = frameId_;
        occupiedNodes.markers[i].header.stamp = ros::Time::now();
        occupiedNodes.markers[i].ns = "occupied_cells";
        occupiedNodes.markers[i].id = i;
        occupiedNodes.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        occupiedNodes.markers[i].scale.x = occupiedNodes.markers[i].scale.y = occupiedNodes.markers[i].scale.z = octree_->getResolution();
        occupiedNodes.markers[i].color.r = 1.0;
        occupiedNodes.markers[i].color.g = 0.0;
        occupiedNodes.markers[i].color.b = 0.0;
        occupiedNodes.markers[i].color.a = 1.0;
        occupiedNodes.markers[i].action = occupiedNodes.markers[i].points.empty() ? visualization_msgs::Marker::DELETE : visualization_msgs::Marker::ADD;
    }

    for (size_t i = 0; i < freeNodes.markers.size(); ++i) {
        freeNodes.markers[i].header.frame_id = frameId_;
        freeNodes.markers[i].header.stamp = ros::Time::now();
        freeNodes.markers[i].ns = "free_cells";
        freeNodes.markers[i].id = i;
        freeNodes.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        freeNodes.markers[i].scale.x = freeNodes.markers[i].scale.y = freeNodes.markers[i].scale.z = octree_->getResolution();
        freeNodes.markers[i].color.r = 0.0;
        freeNodes.markers[i].color.g = 1.0;
        freeNodes.markers[i].color.b = 0.0;
        freeNodes.markers[i].color.a = 1.0;
        freeNodes.markers[i].action = freeNodes.markers[i].points.empty() ? visualization_msgs::Marker::DELETE : visualization_msgs::Marker::ADD;
    }

    occupiedCellsPub_.publish(occupiedNodes);
    freeCellsPub_.publish(freeNodes);
}








int main(int argc, char** argv) {
    ros::init(argc, argv, "custom_octomap_server");

    ros::NodeHandle nh;
    CustomOctomapServer server;

    ros::spin();  // Keep the node running
    return 0;
}
