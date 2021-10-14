#include <sim_vel_control/sim_vel_control.h>

int main(int argc, char** argv)
{
  int i;
  ros::init(argc, argv, "sim_vel_control_node");
  ros::NodeHandle nh;

  SimVelocityControl sim_vel_control(nh); 

  ros::Timer pub_timer = nh.createTimer(ros::Duration(0.01), &SimVelocityControl::PubCallback, &sim_vel_control);

  ros::spin();

}
  
