#include <ros/ros.h>                     
#include <std_msgs/Float64.h>           

#include <agiros_msgs/Reference.h>      // Contains a list of setpoints
#include <agiros_msgs/Setpoint.h>       // Each setpoint has state + command
#include <agiros_msgs/QuadState.h>      // Holds pose, velocity, acceleration
#include <agiros_msgs/Command.h>        // Holds control command info 

#include <Eigen/Dense>     

using Eigen::Vector3d;


// Includes for simulator
// #include <agi/core/simulator_params.h>     // For SimulatorParams
#include "agiros/agisim.hpp"


#include <boost/filesystem.hpp>            // For fs::path


// Includes for controller
#include <agilib/controller/geometric/controller_geo.hpp>
#include <agilib/controller/geometric/geo_params.hpp>
#include <agilib/types/quadrotor.hpp>
// #include <agilib/base/yaml.hpp>


// namespace fs = boost::filesystem;



static inline agi::SimulatorParams loadSimulatorParams(const ros::NodeHandle& nh) {
    std::string simulator_config;
    std::string agi_param_dir;
    std::string ros_param_dir;

    const bool got_config = nh.getParam("simulator_config", simulator_config);
    const bool got_directory = nh.getParam("agi_param_dir", agi_param_dir);
    nh.getParam("ros_param_dir", ros_param_dir);

    if (!got_config || !got_directory) {
        ROS_FATAL("Missing simulator_config or agi_param_dir parameter!");
        ros::shutdown();
    }

    ROS_INFO_STREAM("Simulator Config: " << simulator_config);
    ROS_INFO_STREAM("Agi Param Dir:    " << agi_param_dir);
    ROS_INFO_STREAM("ROS Param Dir:    " << ros_param_dir);

    return agi::SimulatorParams(fs::path(ros_param_dir) / fs::path(simulator_config),
                                agi_param_dir, ros_param_dir);
}


static inline std::shared_ptr<agi::GeometricControllerParams> loadGeoParams(const ros::NodeHandle& nh) {
    std::string controller_config;
    std::string agi_param_dir;
    std::string ros_param_dir;

    nh.getParam("controller_config", controller_config);
    nh.getParam("agi_param_dir", agi_param_dir);
    nh.getParam("ros_param_dir", ros_param_dir);

    ROS_INFO_STREAM("Controller Config: " << controller_config);
    ROS_INFO_STREAM("Agi Param Dir:     " << agi_param_dir);
    ROS_INFO_STREAM("ROS Param Dir:     " << ros_param_dir);

    std::string full_path = (fs::path(ros_param_dir) / fs::path(controller_config)).string();

    agi::Yaml yaml(full_path);
    auto geo_params = std::make_shared<agi::GeometricControllerParams>();
    geo_params->load(yaml);
    return geo_params;
}






// Define the main class that handles energy simulation
class EnergySimNode {
public:
    EnergySimNode(ros::NodeHandle& nh)
        : nh_(nh),                     
            sim_dt_(1.0 / 50.0),         
            total_energy_(0.0)           
    {
        // Subscribe to the trajectory topic (from Moji's global planner)
        traj_sub_ = nh_.subscribe("/kingfisher/agiros_pilot/trajectory", 1, &EnergySimNode::trajectoryCallback, this);

        // Publish energy consumed for trajectory
        energy_pub_ = nh_.advertise<std_msgs::Float64>("/energy_consumed", 1);



        // TODO: check with Leonard if implementation is correct 
        agi::SimulatorParams sim_params = loadSimulatorParams(nh_);     //load params
        // agi_simulator_ = std::make_shared<agi::SimulatorBase>(sim_params);    //create sim instance
        // OR??:
        agi_simulator_ = std::make_shared<agi::QuadrotorSimulator>(sim_params);




        // TODO: Create instances of the controller (ask Leonard)
        auto geo_params = loadGeoParams(nh_);           //load controller params
    

        agi::QuadrotorSimulator sim(sim_params);
        agi::Quadrotor quad = sim.getQuadrotor();
        
        
        
        controller_geo_ = std::make_shared<agi::GeometricController>(quad, geo_params);    //create geo controloer instance



        // Initialise command time to zero
        agi_cmd_.t = 0.0;
    }



private:
    // ROS communication tools
    ros::NodeHandle nh_;                 
    ros::Subscriber traj_sub_;          
    ros::Publisher energy_pub_;         

    // Agilicious components    > here/public, shared_ptr / make_shared
    // std::shared_ptr<agi::SimulatorBase> agi_simulator_;         // Simulates physics
    // or?:
    std::shared_ptr<agi::QuadrotorSimulator> agi_simulator_;
    
    std::shared_ptr<agi::GeometricController> controller_geo_; // Control logic

    
    // Simulator state and input
    agi::Command agi_cmd_;              // Last control command
    agi::QuadState agi_quad_state_;     // Current simulated state of the drone


    double sim_dt_;                     // Simulation dt (in sec)
    double total_energy_;               // Total energy accumulated per trajectory


    // Function to run a single simulator step
    bool step(const Eigen::Vector4d& pi_act_) {
        // Update simulation time
        agi_cmd_.t += sim_dt_;

        // Copy control input to command structure
        agi_cmd_.collective_thrust = pi_act_[0];     // Thrust input
        agi_cmd_.omega = pi_act_.segment<3>(1);      // Body rates (roll/pitch/yaw rate)

    

        // Get the new quadrotor state after simulation
        agi_simulator_->getState(&agi_quad_state_);
        return true;
    }



    // This function is triggered when a trajectory message is received
    void trajectoryCallback(const agiros_msgs::Reference::ConstPtr& msg) {
        total_energy_ = 0.0;                        // Reset energy count

        agi::SetpointVector references;             // Store all waypoints


        // Loop over each Setpoint in the received trajectory
        for (const auto& sp : msg->points) {
            agi::Setpoint ref;

            // Extract position
            // ref.state.p() << sp.state.pose.position.x,
            //                 sp.state.pose.position.y,
            //                 sp.state.pose.position.z;

            // Extract orientation 
            Eigen::Quaterniond q(sp.state.pose.orientation.w,
                                sp.state.pose.orientation.x,
                                sp.state.pose.orientation.y,
                                sp.state.pose.orientation.z);
            ref.state.q(q); // Set orientation in the reference state

 

            ref.state.p = Eigen::Vector3d(sp.state.pose.position.x,
                            sp.state.pose.position.y,
                            sp.state.pose.position.z);

            ref.state.v = Eigen::Vector3d(sp.state.velocity.linear.x,
                            sp.state.velocity.linear.y,
                            sp.state.velocity.linear.z);

            ref.state.a = Eigen::Vector3d(sp.state.acceleration.linear.x,
                            sp.state.acceleration.linear.y,
                            sp.state.acceleration.linear.z);

                

            ref.input.collective_thrust = 9.81;
            ref.input.omega.setZero();
            ref.input.t = agi_cmd_.t;

            
            agi::SetpointVector setpoints;

            // Call the GEO controller to get a command for this reference ref for current state aqi_quad_state and ouput as setpoints pointer
            controller_geo_->getCommand(agi::QuadState(agi_quad_state_), {ref}, &setpoints);


            // Get the first command (but isnt there only one??
            // agi::Command command_geo = setpoints.front().input;
            agi::Command agi_cmd = setpoints.front().input;



            // Validate command: if invalid, set safe default
            if (!agi_cmd_.valid()) {
                agi_cmd_.t = 0;
                agi_cmd_.collective_thrust = 9.81;
                agi_cmd_.omega.setZero();
            }
    
            // Call the simulator to apply this command for one time step
            if (!agi_simulator_->run(agi_cmd, sim_dt_)) {
                ROS_ERROR("Simulator step failed.");
                return; //exit early if sim fails
            }



            agi_cmd_.t += sim_dt_;

            agi_simulator_->getState(&agi_quad_state_);
            

            // !!!!!  TODO: How to get the exact energy from quad state?
            // total_energy_ += agi_quad_state_.mot.pow(3).sum() / 1E9;
            total_energy_ += agi_quad_state_.mot.array().pow(3).sum() / 1E9;

                
        }

        
        // Pub energy topic 
        std_msgs::Float64 energy_msg;
        energy_msg.data = total_energy_;
        energy_pub_.publish(energy_msg);
        // Print energy for debugging


        std_msgs::Float64 test_msg;
        test_msg.data = 42.0;
        energy_pub_.publish(test_msg);
        ROS_INFO("Published test energy value");


        
    }
};



int main(int argc, char** argv) {
    ros::init(argc, argv, "energy_sim_node"); 
    // ros::NodeHandle nh; 
    ros::NodeHandle nh("~");                  
    EnergySimNode node(nh);                   
    ros::spin();                              
    return 0;
}
    
