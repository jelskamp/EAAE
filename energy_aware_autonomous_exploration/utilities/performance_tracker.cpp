#include <ros/ros.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <math.h>

#include <sys/stat.h>
#include <unistd.h>
#include <ctime>
#include <iomanip>
#include <sstream>

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
    std::ofstream log_file_stream;
    double resolution;
    // double map_size = 22.0;     //basic_enclosed_small.world
    
    // double map_size = 25.5;     //simple_no_walls.world  > max side size, 25.5[m]
    // double map_size = 20.5;     //very_simple.world  > max side size, 20.5
    double map_size = 22.5;     //pillars.world  > max side size, 22.5

    // double map_area = 460.44;    // simple_no_walls map area : 460.44 [m]
    // double map_area = 20.5 * 20.0;    // very_simple.world map area
    double map_area = 21.75 * 22.5 - 3*(2 * 2);    // pillars.world map area
    

public:
    ExplorationTracker() {
        
        setupLogging();

        ros::NodeHandle nh;
        octomap_sub = nh.subscribe("/octomap_binary", 1, &ExplorationTracker::octomapCallback, this);
        start_time = ros::Time::now();
        
        // Create and open log file
        std::ofstream file(log_file_path, std::ios::out);
        if (file.is_open()) {
            file << "Time,Explored Area (%)\n";
            file.close();
        }

        ROS_INFO("ExplorationTracker initialized. Logging to: %s", log_file_path.c_str());
    }

    

    void setupLogging() {
        // Get home directory
        std::string home_dir = std::getenv("HOME");
        std::string log_dir = home_dir + "/DATA_exploration";

        // Check if directory exists
        struct stat info;
        if (stat(log_dir.c_str(), &info) != 0) {
            ROS_INFO("Log directory does not exist. Creating it...");
            if (mkdir(log_dir.c_str(), 0777) != 0) {
                ROS_ERROR("Failed to create log directory: %s", log_dir.c_str());
                return;
            } else {
                ROS_INFO("Directory created: %s", log_dir.c_str());
            }
        } else {
            ROS_INFO("Log directory exists: %s", log_dir.c_str());
        }

        // Generate timestamped filename
        std::time_t now = std::time(nullptr);
        std::stringstream timestamp_stream;
        timestamp_stream << std::put_time(std::localtime(&now), "%Y-%m-%d_%H-%M-%S");

        // std::string log_file_path = log_dir + "/exploration_log_" + timestamp_stream.str() + ".txt";
        log_file_path = log_dir + "/exploration_log_" + timestamp_stream.str() + ".csv";


        // Open log file
        log_file_stream.open(log_file_path.c_str(), std::ios::out);

        if (!log_file_stream.is_open()) {
            // THIS IS TRUE!!! > So log file doesnt open???
            ROS_ERROR("Failed to open log file: %s", log_file_path.c_str());
        } else {
            // ROS_INFO("Logging to file: %s", log_file_path.c_str());
        }
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
        int grid_size = static_cast<int>(map_size / resolution); // Assuming a 20x20m map
        std::vector<std::vector<bool>> explored(grid_size, std::vector<bool>(grid_size, false));

        // int total_xy_voxels = grid_size * grid_size;
        int total_xy_voxels = map_area / pow(resolution, 2);
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


        // Log data
        logData(elapsed_time, explored_percentage);


        // Plot data if X% is explored (plot builds up over time)
        if (explored_percentage >= 20) {
            // Plot data       > CHECK IF DATA IS LOGGED (NEEDED FOR PLOT INPUT!)
            plotData();
        }

        
    }

    void logData(double time, double percentage) {
        std::ofstream file(log_file_path, std::ios::app);
        if (file.is_open()) {
            file << time << "," << percentage << "\n";
            // file.close();
            // ROS_INFO("Successfully logged data to file.");
        } else {
            // THIS HAPPENS: so again, cant open log file!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            // ROS_ERROR("Failed to open log file: %s", log_file_path.c_str());
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
        // plt::save("exploration_plot.png");
    }



};

int main(int argc, char** argv) {
    ros::init(argc, argv, "exploration_tracker");
    ExplorationTracker tracker;
    ros::spin();
    return 0;
}







