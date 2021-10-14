#include <sim_vel_control/sim_vel_control.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

SimVelocityControl::SimVelocityControl(ros::NodeHandle& nh): 
    nh(nh), kAccel_rps(1.0)
{
    ros::NodeHandle nh2("~"); // 'private' node handle lets you read in params by relative name
    
    if(!nh2.getParam("joint_names", joint_names)){
        ROS_ERROR("SimVelocityControl: No joint name parameter found");
    }
   
    std::vector<double> initial_joint_position;
    if(!nh2.getParam("initial_joint_position", initial_joint_position)){
        ROS_INFO("SimVelocityControl: No initial_joint_states name parameter found");
    }
    InitializeState(joint_names.size(), initial_joint_position);
    
    vel_sub = nh.subscribe("/cmd_vel",1, &SimVelocityControl::VelCmdCallback, this);
    state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
 }

void SimVelocityControl::VelCmdCallback( const std_msgs::Float64MultiArray& vel_cmd )
{
    vel_goal = vel_cmd.data;
}

void SimVelocityControl::PubCallback( const ros::TimerEvent& )
{ 
    // TODO: If not for simulation I would also put a check here and set velocity to zero if a new
    // command did not arrive in the past x ms.

    ros::Time curr_time = ros::Time::now();
    joint_state.header.stamp = curr_time;

    const double dt = (curr_time-prev_time).toSec();
    const int num_joints = vel_goal.size();

    // Calculate new position
    for(size_t i=0; i<num_joints; i++)
    {
        vel_curr[i] = UpdateVel(vel_prev[i], vel_goal[i], dt);
        vel_curr[i] = AdjustVelForGoalReached(vel_curr[i], vel_prev[i], vel_goal[i]);

        pos_curr[i] = pos_prev[i] + dt * vel_curr[i];
    }

    // Prepare values for next iteration
    prev_time = curr_time;
    prev_joint_state = joint_state;
    
    state_pub.publish(joint_state);
}

double SimVelocityControl::UpdateVel(const double& prev, const double& goal, const double& dt)
{
    if (prev == goal)
        {return goal;}
    if (prev < goal)
        {return prev + (kAccel_rps * dt);}
        
    return prev - (kAccel_rps * dt);
}

double SimVelocityControl::AdjustVelForGoalReached(const double& curr, const double& prev, const double& goal)
{
    if ((prev < goal) && (curr > goal))
       {return goal;}
    if ((prev > goal) && (curr < goal))
       {return goal;}  

    return curr;
}

double SimVelocityControl::InitializeState(size_t num_joints, std::vector<double>& init_position)
{
    std::vector<double> zeros(num_joints, 0.0);

    for (auto& x: init_position){
    std::cout << "init_position: "<< x << std::endl;}
    if(init_position.size() != num_joints){
        
        init_position = zeros;
    }
    
    joint_state.position = init_position;
    joint_state.velocity = zeros;
    joint_state.name = joint_names;
    joint_state.header.stamp = ros::Time::now();

    prev_joint_state = joint_state;
}
