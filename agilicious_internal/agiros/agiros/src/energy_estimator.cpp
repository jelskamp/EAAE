#include <ros/ros.h>                     
#include <std_msgs/Float64.h>           

#include <agiros_msgs/Reference.h>      // Contains a list of setpoints
#include <agiros_msgs/Setpoint.h>       // Each setpoint has state + command
#include <agiros_msgs/QuadState.h>      // Holds pose, velocity, acceleration
#include <agiros_msgs/Command.h>        // Holds control command info 

#include <Eigen/Dense>                  







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



        // TODO: Create instances of the controller and simulator , like this?? (shared_ptr?)
        controller_geo_ = std::make_shared<agi::GeometricController>();
        agi_simulator_ = std::make_shared<agi::SimulatorBase>();



        // Initialise command time to zero
        agi_cmd_.t = 0.0;
    }

private:
    // ROS communication tools
    ros::NodeHandle nh_;                 
    ros::Subscriber traj_sub_;          
    ros::Publisher energy_pub_;         

    // Agilicious components    > here/public, shared_ptr / make_shared
    std::shared_ptr<agi::GeometricController> controller_geo_; // Control logic
    std::shared_ptr<agi::SimulatorBase> agi_simulator_;         // Simulates physics

    
    // Simulator state and input
    agi::Command agi_cmd_;              // Last control command
    agi::QuadState agi_quad_state_;     // Current simulated state of the drone

    // Timing and tracking
    double sim_dt_;                     // Simulation time step (in sec)
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
            ref.state.p() << sp.state.pose.position.x,
                            sp.state.pose.position.y,
                            sp.state.pose.position.z;

            // Extract orientation 
            Eigen::Quaterniond q(sp.state.pose.orientation.w,
                                sp.state.pose.orientation.x,
                                sp.state.pose.orientation.y,
                                sp.state.pose.orientation.z);
            ref.state.q(q); // Set orientation in the reference state

            // Set velocity vector
            ref.state.v() << sp.state.velocity.linear.x,
                            sp.state.velocity.linear.y,
                            sp.state.velocity.linear.z;

            // Set acceleration 
            ref.state.a() << sp.state.acceleration.linear.x,
                            sp.state.acceleration.linear.y,
                            sp.state.acceleration.linear.z;
'
'
            // Dummy values req. for Agilcious ???
            ref.input.collective_thrust = 9.81;
            ref.input.omega.setZero();
            ref.input.t = agi_cmd_.t;

            references.push_back(ref);
        }

        // Loop through each reference state and simulate
        for (const auto& ref : references) {
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
            if (!agi_simulator_->run(agi_cmd_, sim_dt_)) {
            ROS_ERROR("Simulator step failed.");
            return false;
            }

            agi.cmd.t += sim_dt_;

            agi_simulator_->getState(&agi_quad_state_);
            

            // !!!!!  TODO: How to get the exact energy from quad state?
            total_energy_ += agi_quad_state_.getEnergyConsumed();

    


        }

        total_energy = agi_quad_state_.e
        
        // Pub energy topic 
        std_msgs::Float64 energy_msg;
        energy_msg.data = total_energy_;
        energy_pub_.publish(energy_msg);
        // Print energy for debugging
        // ROS_INFO("Trajectory consumed %.2f J of energy", total_energy_);
    }
};



int main(int argc, char** argv) {
    ros::init(argc, argv, "energy_sim_node"); 
    ros::NodeHandle nh;                       

    EnergySimNode node(nh);                   

    ros::spin();                              
    return 0;
}
    
