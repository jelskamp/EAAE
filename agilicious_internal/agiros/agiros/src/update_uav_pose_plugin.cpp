#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/PoseStamped.h>
#include <agiros_msgs/QuadState.h>
#include <mutex>

namespace gazebo
{
    class UpdateUAVPosePlugin : public ModelPlugin
    {
    private:
        physics::ModelPtr model;
        ros::NodeHandle* nh;
        ros::Subscriber state_subscriber;
        ignition::math::Pose3d current_pose;
        std::mutex pose_mutex;

    public:
        UpdateUAVPosePlugin() : ModelPlugin() {}

        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
        {
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.");
                return;
            }


            this->model = _model;
            this->nh = new ros::NodeHandle();

            // Subscribe to Agilicious state topic
            this->state_subscriber = this->nh->subscribe("/kingfisher/agiros_pilot/state", 10, &UpdateUAVPosePlugin::OnStateReceived, this);

            ROS_INFO("UpdateUAVPosePlugin loaded and subscribed to /kingfisher/agiros_pilot/state");
        }

        void OnStateReceived(const agiros_msgs::QuadState::ConstPtr &msg)
        {
            std::lock_guard<std::mutex> lock(pose_mutex);
            this->current_pose.Set(
                ignition::math::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
                ignition::math::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z)
            );

            // ROS_INFO("Received Pose Update: x=%.2f, y=%.2f, z=%.2f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            // ROS_INFO("Applying Pose to Gazebo Model");




            // Apply new pose to Gazebo WITHOUT debug statements
            // this->model->SetLinkWorldPose(this->current_pose, this->model->GetLink("base_link"));
            
            
            // Apply new pose to Gazebo WITH debug statements
            auto base_link = this->model->GetLink("kingfisher/base_link");

	    if (!this->model) {
    		ROS_ERROR("UpdateUAVPosePlugin: Model pointer is null!");
    		return;
	    }

	    if (!base_link) {
    		ROS_ERROR("UpdateUAVPosePlugin: base_link not found! Check the UAV model.");
    		return;
	    }

	    // Apply new pose to Gazebo
	    this->model->SetLinkWorldPose(this->current_pose, base_link);
            
            
            
            
            
        }

        ~UpdateUAVPosePlugin()
        {
            delete this->nh;
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(UpdateUAVPosePlugin)
}
