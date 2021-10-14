#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

//TODO: Implement correctly (with a header, etc.)

struct Marker{
    ros::Publisher pub;
    std::string name;
    Marker(ros::Publisher& p, std::string n): pub(p), name(n){}
};

int main(int argc, char** argv){
  ros::init(argc, argv, "sim");

  ros::NodeHandle node("~");
  std::vector<std::string> marker_names;
  
  if(!node.getParam("marker_names", marker_names))
  {
    ROS_ERROR("SimMarkerListener: No marker name parameter found");
  }

  std::vector<Marker> marker_list;

  for (auto& name : marker_names)
  {
    ros::Publisher pub = node.advertise<geometry_msgs::TransformStamped>("/NDI/"+name+"/measured_cp",1);
    marker_list.push_back(Marker(pub,name));
  }


  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(50.0);
  while (node.ok()){
    for(auto& marker : marker_list ){
      geometry_msgs::TransformStamped transformStamped;
      try{
        //TODO: could implement an actual frame for the polaris (and also determine if it is in frame, add noise, etc.)
        //TODO: could include a 'wait for transform' to remove warning
        transformStamped = tfBuffer.lookupTransform("world", marker.name, ros::Time(0));
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
      transformStamped.transform.translation.x;// *= 1000.0;
      transformStamped.transform.translation.y;// *= 1000.0;
      transformStamped.transform.translation.z;// *= 1000.0;

      marker.pub.publish(transformStamped);
    }
    rate.sleep();
  }
  return 0;
};