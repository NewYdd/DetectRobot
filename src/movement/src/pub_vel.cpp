/*This is for testing the wheel_controller
 *publish:   /cmd_vel
 *subscribe: none
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc,char** argv){
 ros::init(argc,argv,"pub_vel");
 ros::NodeHandle nh;
 ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
 ros::Rate rate(10);
 double i=0.1;
 while (ros::ok()){
   geometry_msgs::Twist vel;
   vel.linear.x=0.3+i;
   vel.angular.z=0.4+i;
   pub.publish(vel);
   ROS_INFO("linera %f, angular %f",vel.linear.x,vel.angular.z);
   i+=0.1;
   rate.sleep();
 }

 return 0;
}
