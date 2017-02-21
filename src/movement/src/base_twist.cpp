/*This node will convert the linear and angular velocity command to individual 
motor velocity target.The target velocity are published at a rate of the ~rate Hz 
and pulish timeout_ticks times velocity after the twist message stop*/


/**Note:  since the same slide wheel has same speed ,so just two wheel variables
          the rate of publish the wheelVelocity depends on the rate of the cmd_vel
**/

/* publishing topics: /wheel_target(movement/wheelVelocity)
 * subscribing topics: /cmd_vel(geometry_msgs/Twist)
*/


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <movement/wheelVelocity.h>

class Robot
{
	public:     /*the parameter of the robot*/
              //double k;     //the distance between the front and back wheel
               double l;      //the distance between the right and left wheel
               double r;      //the radius of wheel;
               double i;      //the reduce rate
               Robot(double ll,double rr,double ii){l=ll;r=rr;i=ii;}

};

class Transform
{
 public : Transform(const ros::Publisher &transform_pub):wheelVel_pub(transform_pub){}
 
          void velCallback(const geometry_msgs::Twist &msg){
          	double lin_vel=msg.linear.x;
          	double ang_vel=msg.angular.z;
          	movement::wheelVelocity vel;
          	Robot robot(0.2,11,22);
          	vel.lwheel  =lin_vel-ang_vel*robot.l/2;
          	vel.rwheel  =lin_vel+ang_vel*robot.l/2;
          	wheelVel_pub.publish(vel);
          	ROS_INFO("the wheel   %f %f",vel.lwheel,vel.rwheel);
          }
 private: ros::Publisher wheelVel_pub; 
      


};


int main(int argc,char** argv){
	ros::init(argc,argv,"baseTwist");
	ros::NodeHandle nh;
	ros::Publisher pub=nh.advertise<movement::wheelVelocity>("/wheel_target",1); 
	Transform transor(pub);
	ros::Subscriber sub=nh.subscribe("/cmd_vel",10,&Transform::velCallback,&transor);
	ros::spin();
	return 0;
}
