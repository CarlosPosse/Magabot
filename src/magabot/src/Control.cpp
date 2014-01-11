#include<math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "magabot/motorSpeed.h"

// This node converts geometry_msgs::Twist from another source to ticks/3.32ms for arduino

#define nodeName "MotorControl" // Node name

#define pubName "motorSpeed" // Topic name to publish individual motor speed for arduino (ticks/3.32ms)
#define pubQueue 1000 // Size of publisher topic queue
#define pubRate 80 // Publishing Rate (Hz)

#define subName "cmd_vel" // Topic name to subscribe velocity commands from another source (geometry_msgs::Twist)
#define subQueue 1000 // Size of subscriber topic queue

#define PI 3.14159

#define Dist 0.355 // Distance between front wheels (meters)
#define R 0.0926/2 // Radius of front wheels (meters)
#define Rev 3900 // Encoder ticks in a wheel revolution (ticks)

#define cte_motor Rev*0.00332/(2*PI*R) // Mathematical conversion from m/s to ticks/3.32ms

class Listener{
	private:
		// Individual motor speed variables
		int leftMotor;
		int rightMotor;
		
		// Message to publish
		magabot::motorSpeed speed;
		
		// Geometry velocity variables
		double vx;
		double vy;
		double wz;	
		
		// Topic variables
		ros::Subscriber subscriber;
  	ros::Publisher publisher;

		// Topic functions
		void publishMessage();
		void callback(const geometry_msgs::Twist::ConstPtr& msg);

	public:
		Listener();
		
		void init(int argc, char **argv);	
  	void run();
};

Listener::Listener(){
	// Listener instantiation, reset the variables
	leftMotor = 0;
	rightMotor = 0;

	vx = 0.0;
	vy = 0.0;
	wz = 0.0;	
}

void Listener::init(int argc, char **argv){
	// Initiate the ROS Node and NodeHandle
	ros::init(argc, argv, nodeName);
	ros::NodeHandle handle;
	
	// Initiate the subscriber and publisher
	subscriber = handle.subscribe(subName, subQueue, &Listener::callback, this);
	publisher = handle.advertise<magabot::motorSpeed>(pubName, pubQueue);
}

void Listener::callback(const geometry_msgs::Twist::ConstPtr& msg){
  // Update the velocity values from the subscribed topic
	vx = msg->linear.x;
	vy = msg->linear.y;
	wz = msg->angular.z;
	
	// Convert the Twist values to individual velocity
	rightMotor = (int)((vx - (wz * Dist/2)) * cte_motor);
	leftMotor = (int)((vx + (wz * Dist/2)) * cte_motor);
}

void Listener::publishMessage(){
	// Build and publish the Speed message for the Arduino
	speed.motorRight = leftMotor;
	speed.motorLeft = rightMotor;
	
	publisher.publish(speed);
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

