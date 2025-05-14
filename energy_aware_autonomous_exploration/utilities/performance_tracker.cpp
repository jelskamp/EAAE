#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <vector>
#include <fstream>
// #include <filesystem>
#include <sys/stat.h>
#include <sys/types.h>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

class ExplorationTracker {
private:
    ros::Subscriber octomap_sub;
    ros::Time start_time;
    std::vector<double> time_data;
    std::vector<double> exploration_data;
    std::string log_file_path;
    double resolution;

    

public:
    ExplorationTracker() {
        ros::NodeHandle nh;
        octomap_sub = nh.subscribe("/octomap_binary", 1, &ExplorationTracker::octomapCallback, this);
        start_time = ros::Time::now();
        
        // Ensure "data" directory exists
        // std::filesystem::create_directories("data");
        mkdir("data", 0777);
        // mkdir("data");
        log_file_path = "data/exploration_log.csv";
        
        // Create and open log file
        std::ofstream file(log_file_path, std::ios::out);
        if (file.is_open()) {
            file << "Time,Explored Area (%)\n";
            file.close();
        }
        // ROS_INFO("");
        // ROS_INFO("");
        // ROS_INFO("ExplorationTracker initialized. Logging to: %s", log_file_path.c_str());
        // ROS_INFO("");
        // ROS_INFO("");
        // ROS_INFO("");
        // ROS_INFO("");
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
        // Convert to OcTree
        std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg)));
        if (!octree) {
            ROS_ERROR("Failed to convert octomap message to OcTree.");
            return;
        }

        // Get map resolution and boundaries
        double resolution = octree->getResolution();
        int grid_size = static_cast<int>(20.0 / resolution); // Assuming a 20x20m map
        std::vector<std::vector<bool>> explored(grid_size, std::vector<bool>(grid_size, false));
        int total_xy_voxels = grid_size * grid_size;
        int explored_voxels = 0;

        // Iterate through all occupied and free voxels
        for (octomap::OcTree::iterator it = octree->begin(); it != octree->end(); ++it) {
            if (it.getDepth() != octree->getTreeDepth()) continue; // Consider only leaf nodes
            int x_idx = static_cast<int>((it.getX() + 10.0) / resolution);
            int y_idx = static_cast<int>((it.getY() + 10.0) / resolution);
            if (x_idx >= 0 && x_idx < grid_size && y_idx >= 0 && y_idx < grid_size) {
                if (!explored[x_idx][y_idx]) {
                    explored[x_idx][y_idx] = true;
                    explored_voxels++;
                }
            }
        }

        // Calculate exploration percentage
        double explored_percentage = (explored_voxels / static_cast<double>(total_xy_voxels)) * 100.0;
        double elapsed_time = (ros::Time::now() - start_time).toSec();
        // ROS_INFO("");
        // ROS_INFO("");
        // ROS_INFO("");
        // ROS_INFO("Logging data: Time: %.2f sec, Explored: %.2f%%", elapsed_time, explored_percentage);
        // ROS_INFO("");
        // ROS_INFO("");


        // Log data
        // logData(elapsed_time, explored_percentage);


        // Plot data if X% is explored (plot builds up over time)
        if (explored_percentage >= 100) {
            // ROS_INFO("");
            // ROS_INFO("");
            // ROS_INFO("......... PLOT MADE .............");
            // ROS_INFO("with resolution %f", resolution);
            // ROS_INFO("");
            // ROS_INFO("");
            // Plot data
            plotData();
        }

        
    }

    void logData(double time, double percentage) {
        std::ofstream file(log_file_path, std::ios::app);
        if (file.is_open()) {
            file << time << "," << percentage << "\n";
            file.close();
            // ROS_INFO("");
            // ROS_INFO("");
            // ROS_INFO("");
            // ROS_INFO("Successfully logged data to file.");
            // ROS_INFO("");
            // ROS_INFO("");
        } else {
            // ROS_INFO("");
            // ROS_INFO("");
            // ROS_ERROR("Failed to open log file: %s", log_file_path.c_str());
            // ROS_INFO("");
            // ROS_INFO("");
            // ROS_INFO("");
        }
        time_data.push_back(time);
        exploration_data.push_back(percentage);
    }


    void plotData() {
        plt::clf();
        plt::plot(time_data, exploration_data, "b-");
        plt::xlabel("Time (s)");
        plt::ylabel("Explored Area (%)");
        plt::title("Exploration Progress");
        plt::grid(true);
        plt::pause(0.01);
        plt::save("exploration_plot.png");
    }




};

int main(int argc, char** argv) {
    ros::init(argc, argv, "exploration_tracker");
    ExplorationTracker tracker;
    ros::spin();
    return 0;
}
