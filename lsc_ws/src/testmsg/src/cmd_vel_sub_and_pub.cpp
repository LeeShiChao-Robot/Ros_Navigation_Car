#include <ros/ros.h>
#include "std_msgs/String.h"
#include "testmsg/Testmag.h"
#include "geometry_msgs/Twist.h"

#define M_PI 3.14159265358979323846
class cmd_SubscribeAndPublish
{
public:
  cmd_SubscribeAndPublish()
  {

    cmd_vx  = 0;
    cmd_vth = 0;
    v_left  = 0;
    v_right = 0;
    cmd_pub_ = n_.advertise<testmsg::Testmag>("car_speed_from_cmd_vel", 50);
    cmd_sub_ = n_.subscribe("cmd_vel", 1000, &cmd_SubscribeAndPublish::cmd_callback, this);
  }
 
  void cmd_callback(const geometry_msgs::Twist& my_cmd_vel)
  {
    testmsg::Testmag speed;
    cmd_vx  = my_cmd_vel.linear.x;
    cmd_vth = my_cmd_vel.angular.z;
    v_right = cmd_vx + cmd_vth*0.21;
    v_left  = cmd_vx - cmd_vth*0.21;
    //speed.left  = v_left;
    //speed.right = v_right;
    speed.left  = (v_left*1600)/(2*M_PI*0.0625);
    speed.right = (v_right*1600)/(2*M_PI*0.0625);
    cmd_pub_.publish(speed);
  }
 
private:
  ros::NodeHandle n_; 
  ros::Publisher cmd_pub_;
  ros::Subscriber cmd_sub_;
  double cmd_vx;
  double cmd_vth;
  double v_left;
  double v_right;
  
 
};
 
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "cmd_vel_sub_and_pub");
  cmd_SubscribeAndPublish SAPObject;
  ros::spin();
  return 0;
}
