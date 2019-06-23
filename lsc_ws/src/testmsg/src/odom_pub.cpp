#include "ros/ros.h"
#include "std_msgs/String.h"
#include "testmsg/Testmag.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    x = 0.0;
    y = 0.0;
    th = 0.0;

    vx = 0.0;
    vy = 0.0;
    vth = 0.0;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    pub_ = n_.advertise<nav_msgs::Odometry>("odom", 50);
    sub_ = n_.subscribe("car_odomspeed", 1000, &SubscribeAndPublish::callback, this);
  }
 
  void callback(const testmsg::Testmag vel)
  {
     current_time = ros::Time::now();
     vx = (vel.left+vel.right)/2000 ;       //v=（VR+VL）/2
     vth = ((vel.right/1000)-(vel.left/1000))/0.42 ;   //w=(vR-vL)/(2*d)  d为轮子到车中心的距离
     
    double dt = 0.02;
    double delta_x = vx * cos(th)  * dt;
    double delta_y = vx * sin(th)  * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;   
    
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth; 
    
    pub_.publish(odom);
    last_time = current_time;

  }
 
private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_; 
  tf::TransformBroadcaster odom_broadcaster;  
  double x;
  double y;
  double th;

  double vx;
  double vy;
  double vth;
  ros::Time current_time, last_time;

};
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_pub");
  SubscribeAndPublish SAPObject;
  ros::spin(); 
  return 0;
}
