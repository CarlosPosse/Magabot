#include<math.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "magabot/motorClick.h"

#define nodeName "MotorOdometry" // Node name

#define pubName "odom" // Topic name to publish odometry information (nav_msgs::Odometry)
#define pubQueue 1000 // Size of publisher topic queue
#define pubRate 80 // Publishing Rate (Hz)

#define subName "vel" // Topic name to subscribe tick information from arduino (
#define subQueue 1000 // Size of subscriber topic queue

#define odomFrame_id "odom" // Odometry frame id
#define odomChildFrame_id "base_link" // Odometry Child frame id

#define PI 3.14159

#define Dist 0.355 // Distance between front wheels (meters)
#define R 0.0926/2 // Radius of front wheels (meters)
#define Rev 3900 // Encoder ticks in a wheel revolution (ticks)

#define dt 0.1 // Time between position samples (seconds)

#define cte_th 2*PI*R/(Dist*Rev) // Mathematical conversion from ticks to radians
#define cte_xy R*PI/Rev // Mathematical conversion from ticks to meters

class Listener{
	private:
		// Individual motor ticks variables
		int leftMotorTicks;		
		int rightMotorTicks;		
		
		// Geometry velocity variables
		double vx;
		double vy;
		double wz;

		// Distance traveled variables
		double x;
		double y;
		double th;
		
		// Message to publish
		nav_msgs::Odometry odom;
		
		// ROS time variable for odometry message
		ros::Time scanTime;
		
		// Quaternion needed for Odometry
    geometry_msgs::Quaternion odomQuaternion;
		
		// Topic variables
		ros::Subscriber subscriber;
  	ros::Publisher publisher;

		// Topic functions
		void publishMessage();
		void callback(const magabot::motorClick::ConstPtr& msg);

	public:
		Listener();
		
		void init(int argc, char **argv);	
  	void run();
};

Listener::Listener(){
	// Listener instantiation, reset the variables
	leftMotorTicks = 0;		
	rightMotorTicks = 0;		
	
	vx = 0.0;
	vy = 0.0;
	wz = 0.0;

	x = 0.0;
	y = 0.0;
	th = 0.0;
}

void Listener::init(int argc, char **argv){
	// Initiate the ROS Node and NodeHandle
	ros::init(argc, argv, nodeName);
	ros::NodeHandle handle;
	
	// Initiate the subscriber and publisher
	subscriber = handle.subscribe(subName, subQueue, &Listener::callback, this);
	publisher = handle.advertise<nav_msgs::Odometry>(pubName, pubQueue);
}

void Listener::callback(const magabot::motorClick::ConstPtr& msg){
	// Update the ticks values from the subscribed topic
	rightMotorTicks = msg->motorRight;
	leftMotorTicks = msg->motorLeft;
	
	// Convert the individual ticks into position variation
	double delta_th = cte_th * double(rightMotorTicks - leftMotorTicks);
	double delta_x = cte_xy * double(rightMotorTicks + leftMotorTicks) * cos(th);
	double delta_y = cte_xy * double(rightMotorTicks + leftMotorTicks) * sin(th);
	
	// Update the position estimates
	x += delta_x;
	y += delta_y;
	th += delta_th;
	
	// Update the current velocity estimates
	vx = double(delta_x / dt);
	vy = double(delta_y / dt);
	wz = double(delta_th / dt);
	odomQuaternion = tf::createQuaternionMsgFromYaw(th);
}

void Listener::publishMessage(){
	// Build and publish the Odometry message to publish
	
	// Set the time
	scanTime = ros::Time::now();
	
	// Set the header
	odom.header.stamp = scanTime;
  odom.header.frame_id = odomFrame_id;

  // Set the position
  odom.pose.pose.position.x = x;
 	odom.pose.pose.position.y = y;
 	odom.pose.pose.position.z = 0.0;
 	odom.pose.pose.orientation = odomQuaternion;

	// Set the velocity
	odom.child_frame_id = odomChildFrame_id;
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = wz;

	// Set the pose covariance
	odom.pose.covariance[0]  = 0.001;
	odom.pose.covariance[7]  = 0.001;
	odom.pose.covariance[14] = 0.001;
	odom.pose.covariance[21] = 1000;
	odom.pose.covariance[28] = 1000;
	odom.pose.covariance[35] = 1000;
	
	// Set the twist covariance
	odom.twist.covariance = odom.pose.covariance;

	publisher.publish(odom);
}

void Listener::run(){
	ros::Rate loopRate(pubRate);
	
	while (ros::ok()){    
		ros::spinOnce();
		
		publishMessage();

		loopRate.sleep();
	}
}

// Main function
int main(int argc, char **argv)
{
	Listener listener;

	listener.init(argc, argv);

  listener.run();

	return 0;
}

