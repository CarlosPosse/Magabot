#include<math.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "magabot/motorClick.h"

#define nodeName  "MotorOdometry" // nome do nó
#define subName1  "vel" // nome do topico onde recebe ticks do arduino
#define PubName1  "odom" // nome do topico onde publica a odometria

#define odomFrame_id "odom"
#define odomChildFrame_id "base_link"

#define PI 3.14159

#define Dist 0.355 //diametro do robo ou seja distancia entre as rodas
#define R 0.0926/2 //diametro da roda em metros 0,0926 m
#define Rev 3900  //numero total de ticks dos encoders numa volta da roda

#define cte_th 2*PI*(R/(Dist*Rev))
#define cte_xy R*(PI/Rev)

class Listener{
	private:
		//**********************
		int leftMotorTicks;		
		int rightMotorTicks;		
		
		//velocidades medidas pelos encoders do robo
		double vx;
		double vy;
		double wz;

		//distancia total percorrida pelo robo
		double x;
		double y;
		double th;
		
		ros::Time scan_time;
		
		nav_msgs::Odometry odom;		
		
		//since all odometry is 6DOF we'll need a quaternion created from yaw
    	geometry_msgs::Quaternion odom_quat;
		
		//**********************
		ros::Subscriber sub1;
  		ros::Publisher pub1;

		void publishMessage();
		void callback1(const magabot::motorClick::ConstPtr& msg);

	public:
		Listener();
		~Listener();		
		
		void init(int argc, char **argv);	
  		void run();
};
Listener::Listener(){

	leftMotorTicks = 0;		
	rightMotorTicks = 0;		
	
	//velocidades medidas pelos encoders do robo
	vx = 0.0;
	vy = 0.0;
	wz = 0.0;

	//distancia total percorrida pelo robo
	x = 0.0;
	y = 0.0;
	th = 0.0;
}
Listener::~Listener(){
	
}
void Listener::init(int argc, char **argv){	
	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;
	
	sub1 = n.subscribe(subName1, 100, &Listener::callback1, this);
	pub1 = n.advertise<nav_msgs::Odometry>(PubName1, 100);
}
void Listener::callback1(const magabot::motorClick::ConstPtr& msg){
	//**********************
	rightMotorTicks = msg->motorRight;
	leftMotorTicks = msg->motorLeft;
	
	double dt = 0.1;
				
	double delta_th = cte_th*double(rightMotorTicks-leftMotorTicks);
	double delta_x = cte_xy*double(rightMotorTicks+leftMotorTicks)*cos(th);
	double delta_y = cte_xy*double(rightMotorTicks+leftMotorTicks)*sin(th);
	
	x += delta_x;
	y += delta_y;
	th += delta_th;		

	vx = double(delta_x/dt);
	vy = double(delta_y/dt);
	wz = double(delta_th/dt);
	odom_quat = tf::createQuaternionMsgFromYaw(th);

	//**********************
}
void Listener::publishMessage(){
	//**********************	
	
	scan_time = ros::Time::now();
	odom.header.stamp = scan_time;
    odom.header.frame_id = odomFrame_id;

    //set the position
    odom.pose.pose.position.x = x;
 	odom.pose.pose.position.y = y;
 	odom.pose.pose.position.z = 0.0;
 	odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = odomChildFrame_id;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = wz;

	//set the covariance
	odom.pose.covariance[0]  = 0.001;
	odom.pose.covariance[7]  = 0.001;
	odom.pose.covariance[14] = 0.001;
	odom.pose.covariance[21] = 1000;
	odom.pose.covariance[28] = 1000;
	odom.pose.covariance[35] = 1000;

	odom.twist.covariance = odom.pose.covariance;

	//**********************
    pub1.publish(odom);
}
void Listener::run(){
	ros::Rate loop_rate(80);
  	while (ros::ok()){    
    	ros::spinOnce();
		
		//**********************
		publishMessage();

		//**********************
    	loop_rate.sleep();
  	}
}
//##########################################################
int main(int argc, char **argv)
{
	Listener listener;

	listener.init(argc, argv);

    listener.run();

	return 0;
}
