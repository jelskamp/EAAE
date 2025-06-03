#include <ros/ros.h>
#include <agiros_msgs/QuadState.h>
#include <vector>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>

class EnergyLogger {
private:
    ros::Subscriber motor_state_sub;
    ros::Time last_time;
    std::string log_file_path;
    std::ofstream log_file_stream;
    double total_energy = 0.0;

    ros::Time start_time;

public:
    EnergyLogger() {
        ros::NodeHandle nh;
        motor_state_sub = nh.subscribe("/kingfisher/agiros_pilot/state", 1, &EnergyLogger::energyCallback, this);
        last_time = ros::Time::now();

        start_time = ros::Time::now();

        setupLogging();
    }

    void setupLogging() {
        std::string home_dir = std::getenv("HOME");
        std::string log_dir = home_dir + "/DATA_energy";

        struct stat info;
        if (stat(log_dir.c_str(), &info) != 0) {
            if (mkdir(log_dir.c_str(), 0777) != 0) {
                ROS_ERROR("Failed to create log directory: %s", log_dir.c_str());
                return;
            } else {
                ROS_INFO("Directory created: %s", log_dir.c_str());
            }
        }

        std::time_t now = std::time(nullptr);
        std::stringstream timestamp_stream;
        timestamp_stream << std::put_time(std::localtime(&now), "%Y-%m-%d_%H-%M-%S");

        log_file_path = log_dir + "/energy_log_" + timestamp_stream.str() + ".csv";

        log_file_stream.open(log_file_path.c_str(), std::ios::out);
        if (log_file_stream.is_open()) {
            log_file_stream << "Time (s),Motor1 (rad/s),Motor2,Motor3,Motor4,Total Energy (J)\n";
            ROS_INFO("EnergyLogger initialized. Logging to: %s", log_file_path.c_str());
        } else {
            ROS_ERROR("Failed to open log file: %s", log_file_path.c_str());
        }
    }

    void energyCallback(const agiros_msgs::QuadState::ConstPtr& msg) {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        last_time = current_time;

        if (dt <= 0.0 || dt > 1.0) return; // Ignore large or zero jumps

        std::vector<double> motors = msg->motors;
        if (motors.size() != 4) {
            ROS_WARN("Expected 4 motor speeds, got %zu", motors.size());
            return;
        }

        double power_total = 0.0;
        for (int i = 0; i < 4; ++i) {
            double w = motors[i];
            double P = 6.088e-3 * w + 1.875e-8 * std::pow(w, 3) + 7.700e-20 * std::pow(w, 6);
            power_total += P;
        }

        double energy_step = power_total * dt;
        total_energy += energy_step;

        // TODO: MAKE SURE THIS IS CORRECT:
        // double elapsed_time = (current_time - ros::Time(msg->header.stamp)).toSec();
        // dobule elapsed_time = current_time;
        double elapsed_time = (ros::Time::now() - start_time).toSec();

        if (log_file_stream.is_open()) {
            log_file_stream << std::fixed << std::setprecision(3)
                            << elapsed_time << ","
                            << motors[0] << "," << motors[1] << ","
                            << motors[2] << "," << motors[3] << ","
                            << total_energy << "\n";
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "energy_logger");
    EnergyLogger logger;
    ros::spin();
    return 0;
}






