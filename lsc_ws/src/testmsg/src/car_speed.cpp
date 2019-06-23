#include "ros/ros.h"
#include "std_msgs/String.h"
#include "testmsg/Testmag.h"
float left;
float right;

void chatterCallback(const testmsg::Testmag msg)
{
  ROS_INFO("I heard:");
  std::cout <<msg.left<<std::endl;
  std::cout <<msg.right<< std::endl;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("car_odomspeed", 1000, chatterCallback);
  ros::spin();

  return 0;
}