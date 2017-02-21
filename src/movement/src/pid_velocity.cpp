//function : use PID to make the velocity reaching the desired value

/*subscribe : /wheel_target(movement::wheelVelocity)
              /wheel_real   (movement::wheelVelocity) //the real wheel_velocity
 *publish   : /wheel_command (movement::wheelVelocity) //the output wheel_command
 *parameter : Kp ,Kd, Ki
 
 *version 1.0  2017/02/20/ 16:06:53  
*/
 
 
#include <ros/ros.h>
#include <movement/wheelVelocity.h>

using namespace movement;

class PidVelocity
{
	private : 
	ros::Publisher pub_command;
    ros::Subscriber sub_wheel_target;
    ros::Subscriber sub_wheel_real;
	
	wheelVelocity target;
	wheelVelocity error;
	wheelVelocity error_pre;
	wheelVelocity real;
	
	
	wheelVelocity command;
	wheelVelocity command_max;
	wheelVelocity command_min;
	
	
	
	double Kd;
	double Ki;
	double Kp;
	public :
	ros::Duration cmd_period;// the rate of publishing velocity_cmd
	    PidVelocity(
	        //wheelVelocity command_max,
	        //wheelVelocity command_min,
	  		double Kp=1,
	  	    double kd=0,
	  	    double ki=0
	    );
	    
	    bool init(ros::NodeHandle &controller_nh);
		void targetCallback(const movement::wheelVelocity &msg);
		void realCallback(const movement::wheelVelocity &msg);
        void calVelocity();
		

};
  PidVelocity::PidVelocity(
  	    double Kp,
  	    double Ki,
  	    double Kd
  ):  Kp(Kp),
      Kd(Kd),
      Ki(Ki) 
  {}
  bool PidVelocity::init(  ros::NodeHandle &controller_nh)
 {  
    double publish_rate;
    controller_nh.param("cmd_pub_rate",publish_rate,10.0);// default 50hz
    cmd_period=ros::Duration(1.0/publish_rate);
    
    //get the pid from the yaml
    controller_nh.param("Kp",Kp,Kp);
    controller_nh.param("Kd",Kd,Kd);
    controller_nh.param("Ki",Ki,Ki);
    
    //get the command_max && command_min
    double leftmax,rightmax;
    controller_nh.param("left_max",leftmax,100.0);
    controller_nh.param("right_max",rightmax,100.0);
    command_max.lwheel=leftmax;command_max.rwheel=rightmax;
     
    double leftmin,rightmin;
    controller_nh.param("left_min",leftmin,0.);
    controller_nh.param("right_min",rightmin,0.);
    command_min.lwheel=leftmin;command_min.rwheel=rightmin;
        
    
 	sub_wheel_target=controller_nh.subscribe("/wheel_target", 1, &PidVelocity::targetCallback,this);
    sub_wheel_real=controller_nh.subscribe("/wheel_real", 1, &PidVelocity::realCallback,this);
    pub_command=controller_nh.advertise<movement::wheelVelocity>("/wheel_command",1);
 	return true;
 }   

  void PidVelocity::targetCallback(const movement::wheelVelocity& msg)
 {  
	target.lwheel=msg.lwheel;
	target.rwheel=msg.rwheel;
 }
 
  void PidVelocity::realCallback(const movement::wheelVelocity& msg)
 {  
	real.lwheel=msg.lwheel;
	real.rwheel=msg.rwheel;
 }
  void PidVelocity::calVelocity()
 {   
    command.lwheel=0; command.rwheel=0;
//left wheel
    error.lwheel = target.lwheel - real.lwheel;
    double derror_l=error.lwheel - error_pre.lwheel;
    command.lwheel = Kp*error.lwheel +       Kd*derror_l  ;
    
   //right wheel
    error.rwheel = target.rwheel - real.rwheel;
    double derror_r=error.rwheel - error_pre.rwheel;
    command.rwheel = Kp*error.rwheel +       Kd*derror_r  ;
    
    error_pre.lwheel=error.lwheel; error_pre.rwheel=error.rwheel;
   // limit the max && min velocity_cmd
  	if(command.lwheel>command_max.lwheel)
   		command.lwheel=command_max.lwheel;
   	if(command.lwheel<command_min.lwheel)
   		command.lwheel=command_min.lwheel;
   		
   	if(command.rwheel>command_max.rwheel)
   		command.rwheel=command_max.rwheel;	
   	if(command.rwheel<command_min.rwheel)
   		command.rwheel=command_min.rwheel;
   		
   	if(target.lwheel==0&&target.rwheel==0)
   	{
    	command.lwheel=0;command.rwheel=0;	
    }

   //publish the velocity_cmd
    ROS_INFO("command %f %f %f",command_max.lwheel,command.lwheel,command.rwheel);
    pub_command.publish(command);
 }
 
 
 
 
 int main(int argc,char **argv)
 {
 	ros::init(argc,argv,"pid_velocity");
 	ros::NodeHandle nh;
 	PidVelocity pid;
 	pid.init(nh);
 	
 	ros::Duration period=pid.cmd_period;
 	ros::Rate rate(1/period.toSec());
 	ROS_INFO_STREAM("rate"<<1/period.toSec());
 	while(ros::ok)
 	{
		pid.calVelocity();
		ros::spinOnce();
		rate.sleep(); 	
 	}

 	return 0;
 }
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
