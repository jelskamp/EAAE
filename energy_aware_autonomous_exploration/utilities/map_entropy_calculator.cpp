// #include <ros/ros.h>
// #include <octomap_msgs/Octomap.h>
// #include <octomap/OcTree.h>
// #include <octomap_msgs/conversions.h>
// #include <vector>
// #include <fstream>
// // #include <filesystem>
// #include <sys/stat.h>
// #include <sys/types.h>
// #include "matplotlibcpp.h"
// #include <cmath>
// namespace plt = matplotlibcpp;


// class EntropyCalculator {
// private:
//     ros::Subscriber octomap_sub;
//     ros::Time start_time;
//     std::string log_file_path;
//     double resolution;
//     // INPUT MAP SIZE:
//     double map_size = 20.0;
    

// public:
//     EntropyCalculator() {
//         ros::NodeHandle nh;
//         octomap_sub = nh.subscribe("/octomap_full", 1, &EntropyCalculator::octomapCallback, this);
//         start_time = ros::Time::now();
        
//         // Ensure "data" directory exists
//         bool dir = mkdir("home/data_entropy", 0777);
//         log_file_path = "home/data_entropy/entropy_log.csv";
//         if(!dir) {
//             // ROS_ERROR_STREAM("DIR NOT MADE!!!");
//         } else {
//             // ROS_ERROR_STREAM("DIR MADE!!!");
//         }
//         // Create and open log file
//         std::ofstream file(log_file_path, std::ios::out);
//         if (file.is_open()) {
//             file << "Entropy (bits)\n";
//             file.close();
//         }
//     }

//     void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
//         // Convert to OcTree
//         std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*msg)));
//         if (!octree) {
//             ROS_ERROR("Failed to convert octomap message to OcTree.");
//             return;
//         }

//         // Get map resolution and boundaries
//         double resolution = octree->getResolution();
//         int grid_size = static_cast<int>(map_size / resolution); 
//         double total_entropy = 0.0;
//         int amount_of_cells = 0;
//         // Iterate through all occupied and free voxels > SHOULD BE THROUGH ALL VOXELS??s
//         for (octomap::OcTree::iterator it = octree->begin(); it != octree->end(); ++it) {
            
//             double entropy_for_this_voxel;
//             double p = it->getOccupancy();

//             entropy_for_this_voxel = -p * std::log2(p) -(1-p) * std::log2(1-p);
//             total_entropy += entropy_for_this_voxel;
//             amount_of_cells += 1;   // also: max entropy per cell 
//             // ROS_INFO_STREAM("occupancy:" << p);
//             // ROS_ERROR_STREAM(entropy_for_this_voxel);
//         }


//         // Log data
//         logData(total_entropy);

//         // Plot data if X% is explored (plot builds up over time)
//         // if (total_entropy >= 95) {
//         //     plotData();
//         // }        
//     }

//     void logData(double total_entropy) {
//         std::ofstream file(log_file_path, std::ios::app);
//         if (file.is_open()) {
//             file << total_entropy << "\n";
//             file.close();
//         } else {
//             // ROS_ERROR("Failed to open log file: %s", log_file_path.c_str());
//         }
//     }

//     // void plotData() {
//     //     plt::clf();
//     //     plt::plot(time_data, exploration_data, "b-");
//     //     plt::xlabel("Time (s)");
//     //     plt::ylabel("Explored Area (%)");
//     //     plt::title("Exploration Progress");
//     //     plt::grid(true);
//     //     plt::pause(0.01);
//     //     plt::save("exploration_plot.png");
//     // }
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "entropy_calculator");
//     EntropyCalculator calculator;
//     ros::spin();
//     return 0;
// }










// --------------------------------------------------------------------------------------




#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <vector>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>

class EntropyCalculator {
private:
    ros::Subscriber octomap_sub;
    ros::Time start_time;
    std::string log_file_path;
    std::ofstream log_file_stream;
    double map_size = 20.0;

public:
    EntropyCalculator() {
        ros::NodeHandle nh;
        octomap_sub = nh.subscribe("/octomap_full", 1, &EntropyCalculator::octomapCallback, this);
        start_time = ros::Time::now();

        setupLogging();
    }

    void setupLogging() {
        std::string home_dir = std::getenv("HOME");
        std::string log_dir = home_dir + "/DATA_entropy";

        struct stat info;
        if (stat(log_dir.c_str(), &info) != 0) {
            if (mkdir(log_dir.c_str(), 0777) != 0) {
                ROS_ERROR("Failed to create log directory: %s", log_dir.c_str());
                return;
            } else {
                ROS_INFO("Created directory: %s", log_dir.c_str());
            }
        }

        std::time_t now = std::time(nullptr);
        std::stringstream timestamp_stream;
        timestamp_stream << std::put_time(std::localtime(&now), "%Y-%m-%d_%H-%M-%S");

        log_file_path = log_dir + "/entropy_log_" + timestamp_stream.str() + ".csv";

        log_file_stream.open(log_file_path.c_str(), std::ios::out);
        if (log_file_stream.is_open()) {
            log_file_stream << "Time (s),Entropy (bits),Total Cells\n";
            ROS_INFO("EntropyCalculator initialized. Logging to: %s", log_file_path.c_str());
        } else {
            ROS_ERROR("Failed to open log file: %s", log_file_path.c_str());
        }
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
        std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*msg)));
        if (!octree) {
            ROS_ERROR("Failed to convert octomap message to OcTree.");
            return;
        }

        double total_entropy = 0.0;
        int total_cells = 0;

        for (octomap::OcTree::iterator it = octree->begin(); it != octree->end(); ++it) {
            double p = it->getOccupancy();
            double entropy_for_voxel = 0.0;

            if (p > 0.0 && p < 1.0) {
                entropy_for_voxel = -p * std::log2(p) - (1 - p) * std::log2(1 - p);
            } else {
                // Fully known (occupied or free) → entropy is zero
                entropy_for_voxel = 0.0;
            }

            total_entropy += entropy_for_voxel;
            total_cells++;
        }

        double elapsed_time = (ros::Time::now() - start_time).toSec();
        logData(elapsed_time, total_entropy, total_cells);
    }

    void logData(double time, double entropy, int total_cells) {
        if (log_file_stream.is_open()) {
            log_file_stream << time << " , " << entropy << " , " << total_cells << "\n";
        } else {
            ROS_ERROR("Log file stream not open.");
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "entropy_calculator");
    EntropyCalculator calculator;
    ros::spin();
    return 0;
}
