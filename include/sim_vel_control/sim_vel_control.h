#ifndef SIM_VEL_CONTROL_H_
#define SIM_VEL_CONTROL_H_

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

class SimVelocityControl{
  public:
    SimVelocityControl( ros::NodeHandle& nh );
    void VelCmdCallback( const std_msgs::Float64MultiArray& vel_cmd );
    void PubCallback( const ros::TimerEvent& );
  
  private:
    ros::NodeHandle nh; 
    ros::Subscriber vel_sub;
    ros::Publisher state_pub;
    const double kAccel_rps;  // acceleration rad/sec
    std::vector<std::string> joint_names;
    sensor_msgs::JointState joint_state;
    sensor_msgs::JointState prev_joint_state;
    ros::Time prev_time;
    std::vector<double> vel_goal;

    // Define short names for readability
    const std::vector<double>& vel_prev = prev_joint_state.velocity;
    std::vector<double>& vel_curr = joint_state.velocity;
    const std::vector<double>& pos_prev = prev_joint_state.position;
    std::vector<double>& pos_curr = joint_state.position;

    double AdjustVelForGoalReached(const double& curr, const double& prev, const double& goal);

    double UpdateVel(const double& prev, const double& goal, const double& dt);

    double InitializeState(size_t num_joints, std::vector<double>& init_position);

}; 

#endif // SIM_VEL_CONTROL_H_