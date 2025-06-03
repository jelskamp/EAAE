#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <agiros_msgs/QuadState.h>
#include <cmath>

class WaypointPublisher {
public:
    WaypointPublisher() {
        // Subscribers
        best_waypoint_sub_ = nh_.subscribe("/best_waypoint", 10, &WaypointPublisher::bestWaypointCallback, this);
        uav_state_sub_ = nh_.subscribe("/kingfisher/agiros_pilot/state", 10, &WaypointPublisher::uavStateCallback, this);

        // Publishers
        waypoint_reached_pub_ = nh_.advertise<std_msgs::Bool>("/waypoint_reached", 10);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/goal_frontier_waypoint", 10);

        // Timer to check proximity and publish waypoint
        timer_ = nh_.createTimer(ros::Duration(0.5), &WaypointPublisher::timerCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber best_waypoint_sub_;
    ros::Subscriber uav_state_sub_;
    ros::Publisher waypoint_reached_pub_;
    ros::Publisher goal_pub_;
    ros::Timer timer_;

    geometry_msgs::PoseStamped last_goal_;
    agiros_msgs::QuadState uav_state_;

    bool has_goal_ = false;
    bool waypoint_reached_ = false;

    void bestWaypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        last_goal_ = *msg;
        has_goal_ = true;
        waypoint_reached_ = false;  // reset flag when a new goal is received
        ROS_INFO("Received new waypoint goal.");
    }

    void uavStateCallback(const agiros_msgs::QuadState::ConstPtr& msg) {
        uav_state_ = *msg;
    }

    void timerCallback(const ros::TimerEvent&) {
        if (!has_goal_) return;

        // Publish current goal continuously
        goal_pub_.publish(last_goal_);

        // Check distance to target
        double dx = uav_state_.pose.position.x - last_goal_.pose.position.x;
        double dy = uav_state_.pose.position.y - last_goal_.pose.position.y;
        double dz = uav_state_.pose.position.z - last_goal_.pose.position.z;
        double distance = sqrt(dx * dx + dy * dy);

        if (distance < 0.5 && !waypoint_reached_) {
            std_msgs::Bool msg;
            msg.data = true;
            waypoint_reached_pub_.publish(msg);
            waypoint_reached_ = true;
            ROS_INFO("UAV reached waypoint. Published waypoint_reached = true");
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_publisher");
    WaypointPublisher wp;
    ros::spin();
    return 0;
}
