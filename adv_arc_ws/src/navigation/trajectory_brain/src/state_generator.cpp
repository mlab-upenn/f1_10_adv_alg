#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <trajectory_brain/drive_param.h>
#define FALSE 0
#define TRUE 1

union State
{
	struct 
	{
		double sx;
		double sy;
		double theta; 
		double kappa;
		double v;
		double vdes;
		double timestamp;
	};

	double state_value[7];
};

union Parameters 
{
	struct 
	{
		double a;
		double b;
		double c;
		double e;
		double s;
		bool success;
	};

	double param_value[6];


};

/////////////////////////////////////////////////////////////////
// Global variables
/////////////////////////////////////////////////////////////////

// Set update rate
static const int LOOP_RATE = 100; //Hz

// Next state time difference
static const double next_time = 1.00/LOOP_RATE;

double kmax=10;
double kmin=0;

// Global vairable to hold curvature
union Parameters curvature;

// Global variable to hold vehicle state
union State veh; 

// Global variable to keep track of when the last message was recieved:
//double start_time;

// Global variable keep track of current time in this node:
//double current_time;

/////////////////////////////////////////////////////////////////
// Callback function declarations
/////////////////////////////////////////////////////////////////

// Callback function to get control parameters 
void splineCallback(const std_msgs::Float64MultiArray& spline);

// Callback function to get state parameters 
void stateCallback(const std_msgs::Float64MultiArray& state);

/////////////////////////////////////////////////////////////////
// Function Prototypes
/////////////////////////////////////////////////////////////////
union State nextState();

/////////////////////////////////////////////////////////////////
// Main loop
/////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{

	ROS_INFO_STREAM("Command converter begin: ");
	ROS_INFO_STREAM("Loop Rate: " << next_time);

	// Set up ROS
	ros::init(argc, argv, "twist_gen");

	// Make handles
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	// Publish the following topics:
	// Commands
	ros::Publisher pub_teleop = nh.advertise<trajectory_brain::drive_param>("drive_parameters",10);
	trajectory_brain::drive_param msg;

	// Subscribe to the following topics:
	// Curvature parameters and state parameters
	ros::Subscriber spline_parameters = nh.subscribe("spline", 10, splineCallback);
	ros::Subscriber state_parameters = nh.subscribe("state", 10, stateCallback);

	// Setup message to hold commands
	geometry_msgs::TwistStamped twist;

	nav_msgs::Odometry odom;

	// Setup the loop rate in Hz
	ros::Rate loop_rate(LOOP_RATE); 
	static double vdes;

	// Here we go...
	while (ros::ok())
	{
		std_msgs::Bool _lf_stat;
		ros::spinOnce();

		//current_time = ros::Time::now().toSec();
		//double elapsedTime = current_time - start_time;
		msg.angle = -25;
		msg.velocity = 0;

		// Make sure we haven't finished the mission yet
		// Get desired velocity at next state...
		vdes = veh.vdes;
		// This computes the next command
		nextState();

		// Set lateral velocity
		double v_lateral = vdes*sin(veh.theta);

		odom.twist.twist.linear.y=v_lateral;

		// Set desired lateral position
		odom.pose.pose.position.y=veh.sy;

		// Ensure kappa is reasonable
		//veh.kappa = std::min(kmax, veh.kappa);
		//veh.kappa = std::max(kmin, veh.kappa); 
		// Set yaw rate

		// This can be used for steering angle...
		odom.twist.twist.angular.z=vdes*veh.kappa;
		msg.angle = odom.twist.twist.angular.z*(57.29)*(5/30)*20;
		msg.velocity = 10;
		//            if(tele_cmd.steering>5)
		//                tele_cmd.steering = 5;
		//            else if(tele_cmd.steering<-5)
		//                tele_cmd.steering = -5;
		//            tele_cmd.throttle = 0.6;  

		// Publish messages
		//        cmd_velocity_publisher.publish(twist);
		pub_teleop.publish(msg); 
		loop_rate.sleep();
	}
	return 0;
}


/////////////////////////////////////////////////////////////////
// Call back function for state update
/////////////////////////////////////////////////////////////////

void stateCallback(const std_msgs::Float64MultiArray& state)
{
	int i = 0;

	for(i=0;i<6;i++) {
		veh.state_value[i]=state.data[i];
	}

	return;
}

/////////////////////////////////////////////////////////////////
// Callback function for spline update
/////////////////////////////////////////////////////////////////

void splineCallback(const std_msgs::Float64MultiArray& spline)
{
	// Reset elapsed time to 0, if called...
//	start_time = ros::Time::now().toSec();
	int i = 0;

	for(i=0;i<6;i++) {
		curvature.param_value[i]=spline.data[i];
	}
	return;
}

union State nextState()
{
	//double simtime = 0.0;

//	double a = curvature.a;
	double b = curvature.b;
	double c = curvature.c;
	double e = curvature.e;

	double kappa = veh.kappa;
	double theta = veh.theta;
	double v = veh.v;

	kappa = b * v * next_time + c * v * v * next_time * next_time + e * v * v * v * pow(next_time, 3);
	double dtheta = v * kappa * next_time;
	theta = theta + dtheta;

	veh.sx = veh.sx + v * cos(theta) * next_time;
	veh.sy = veh.sy + v * sin(theta) * next_time;
	veh.kappa = kappa;
	veh.theta = theta;

	return veh;
}
