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
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <movement/speed_limiter.h>


//class Robot
//{
//	public:     /*the parameter of the robot*/
//              //double k;     //the distance between the front and back wheel
//               double l;      //the distance between the right and left wheel
//               double r;      //the radius of wheel;
//               double i;      //the reduce rate
//               Robot(double ll,double rr,double ii){l=ll;r=rr;i=ii;}

//};



  class DiffDrive
  { 
  	 private:
    /// Velocity command related:
    struct Commands
    {
      double lin;
      double ang;
      ros::Time stamp;
      Commands() : lin(0.0), ang(0.0), stamp(0.0) {}
    };
    Commands command_struct;
    realtime_tools::RealtimeBuffer<Commands> command_real;
    
    double wheel_separation;
    double wheel_radius;
    double wheel_separation_multiplier;
    double wheel_radius_multiplier;
    double cmd_vel_timeout;
    
    ros::Publisher pub_vel;
    ros::Subscriber sub_command;
    
    
    
    Commands last0_cmd_;
    Commands last1_cmd_;
    SpeedLimiter limiter_ang_;
    SpeedLimiter limiter_lin_;
    
    public :
         ros::Duration pub_period;
    	 DiffDrive();
    	 bool init(  ros::NodeHandle &controller_nh);
    	 void update(const ros::Time& time, const ros::Duration& period);
    	 void cmdVelCallback(const geometry_msgs::Twist& command);
    
  };
  
	DiffDrive::DiffDrive()
    : command_struct()
    , wheel_separation(0.2)
    , wheel_radius(0.0)
    , wheel_separation_multiplier(1.0)
    , wheel_radius_multiplier(1.0)
    , cmd_vel_timeout(0.5)
  {
  }
  	 bool DiffDrive::init(  ros::NodeHandle &controller_nh)
  { 
  	double pub_rate;
  	controller_nh.param("target_pub_rate",pub_rate,50.0);//
  	pub_period=ros::Duration(1.0/pub_rate);
  	
        //robot parameter
  	 controller_nh.param("wheel_separation", wheel_separation, wheel_separation);
  	 controller_nh.param("wheel_radius", wheel_radius, wheel_radius);
  	  	// Velocity and acceleration limits:
    controller_nh.param("linear/x/has_velocity_limits"    , limiter_lin_.has_velocity_limits    , limiter_lin_.has_velocity_limits    );
    controller_nh.param("linear/x/has_acceleration_limits", limiter_lin_.has_acceleration_limits, limiter_lin_.has_acceleration_limits);
    controller_nh.param("linear/x/has_jerk_limits"        , limiter_lin_.has_jerk_limits        , limiter_lin_.has_jerk_limits        );
    controller_nh.param("linear/x/max_velocity"           , limiter_lin_.max_velocity           ,  limiter_lin_.max_velocity          );
    controller_nh.param("linear/x/min_velocity"           , limiter_lin_.min_velocity           , -limiter_lin_.max_velocity          );
    controller_nh.param("linear/x/max_acceleration"       , limiter_lin_.max_acceleration       ,  limiter_lin_.max_acceleration      );
    controller_nh.param("linear/x/min_acceleration"       , limiter_lin_.min_acceleration       , -limiter_lin_.max_acceleration      );
    controller_nh.param("linear/x/max_jerk"               , limiter_lin_.max_jerk               ,  limiter_lin_.max_jerk              );
    controller_nh.param("linear/x/min_jerk"               , limiter_lin_.min_jerk               , -limiter_lin_.max_jerk              );

    controller_nh.param("angular/z/has_velocity_limits"    , limiter_ang_.has_velocity_limits    , limiter_ang_.has_velocity_limits    );
    controller_nh.param("angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits, limiter_ang_.has_acceleration_limits);
    controller_nh.param("angular/z/has_jerk_limits"        , limiter_ang_.has_jerk_limits        , limiter_ang_.has_jerk_limits        );
    controller_nh.param("angular/z/max_velocity"           , limiter_ang_.max_velocity           ,  limiter_ang_.max_velocity          );
    controller_nh.param("angular/z/min_velocity"           , limiter_ang_.min_velocity           , -limiter_ang_.max_velocity          );
    controller_nh.param("angular/z/max_acceleration"       , limiter_ang_.max_acceleration       ,  limiter_ang_.max_acceleration      );
    controller_nh.param("angular/z/min_acceleration"       , limiter_ang_.min_acceleration       , -limiter_ang_.max_acceleration      );
    controller_nh.param("angular/z/max_jerk"               , limiter_ang_.max_jerk               ,  limiter_ang_.max_jerk              );
    controller_nh.param("angular/z/min_jerk"               , limiter_ang_.min_jerk               , -limiter_ang_.max_jerk              );

  	
   // init /cmd_vel subscriber && /wheelVelocity publisher
    sub_command= controller_nh.subscribe("/cmd_vel", 1, &DiffDrive::cmdVelCallback,this);
    pub_vel=controller_nh.advertise<movement::wheelVelocity>("/wheel_target",1);
    return true;
  }
  
  
  
	 void DiffDrive::update(const ros::Time& time, const ros::Duration& period)
  {
    
    //brake if cmd_vel has timeout
    Commands curr_cmd = *(command_real.readFromRT());
    const double dt=(time-curr_cmd.stamp).toSec();
    ROS_INFO("time %f",dt);
    if(dt>cmd_vel_timeout)
    {
    	curr_cmd.lin=0.0;
    	curr_cmd.ang=0.0;
    }
    
    // limit the speed and acceleration
    const double cmd_dt(period.toSec());
    limiter_lin_.limit(curr_cmd.lin, last0_cmd_.lin, last1_cmd_.lin, cmd_dt);
    limiter_ang_.limit(curr_cmd.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);


 
    last1_cmd_ = last0_cmd_;
    last0_cmd_ = curr_cmd;
    // Compute wheels velocities:
    const double vel_left  = (curr_cmd.lin - curr_cmd.ang * wheel_separation / 2.0);
    const double vel_right = (curr_cmd.lin + curr_cmd.ang * wheel_separation / 2.0);
    
    movement::wheelVelocity vel;
    vel.lwheel=vel_left;
    vel.rwheel=vel_right;
    ROS_INFO("%f %f",vel_left,vel_right);
  // Set wheels velocities,and pub this to /wheelVelocity :
    pub_vel.publish(vel);
  }
  

     void DiffDrive::cmdVelCallback(const geometry_msgs::Twist& command)
  {
      command_struct.ang   = command.angular.z;
      command_struct.lin   = command.linear.x;
      command_struct.stamp = ros::Time::now();
      command_real.writeFromNonRT (command_struct);

  }
 

       
int main(int argc,char** argv){
	ros::init(argc,argv,"baseTwist2");
	ros::NodeHandle nh;
	ros::Time time=ros::Time::now();

	
	DiffDrive drive;
	drive.init(nh);
	ros::Duration period=drive.pub_period;//this rate of publishing /wheel_target
	ros::Rate rate(1/period.toSec());
	
	ROS_INFO_STREAM("rate"<<1/period.toSec());
	
	ros::AsyncSpinner spinner(1);
	spinner.start();
	while(ros::ok)
	{   time=ros::Time::now();
		drive.update(time,period);
		rate.sleep();
	}
	spinner.stop();
	return 0;
}
