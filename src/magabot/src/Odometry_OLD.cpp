#include <math.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "magabot/Status.h"

#define _USE_MATH_DEFINES

#define NODE_NAME "Odometry"

#define PUB_NAME "odom"
#define PUB_QUEUE_SIZE 1000
#define PUB_RATE 100

#define SUB_NAME "/magabot/Status"
#define SUB_QUEUE_SIZE 1000

#define ODOM_FRAME_ID "odom"
#define ODOM_CHILD_FRAME_ID "base_link"

#define DISTANCE 0.345
#define RADIUS 0.045
#define TICKS_PER_REV 3900

#define K_TH 2 * M_PI * RADIUS / (DISTANCE * TICKS_PER_REV)
#define K_XY RADIUS * M_PI / TICKS_PER_REV

class Listener
{
	private:
		int left_wheel_ticks;
		int right_wheel_ticks;

		double v_x;
		double v_y;
		double w_z;

		double x;
		double y;
		double th;

		double dt;

		nav_msgs::Odometry odom;

		ros::Time time_stamp;

		geometry_msgs::Quaternion odom_quaternion;

		ros::Subscriber subscriber;
		ros::Publisher publisher;

		void publish();
		void callback(const magabot::Status::ConstPtr& msg);

	public:
		Listener();
		void init(int argc, char **argv);
		void run();
};

Listener::Listener()
{
	left_wheel_ticks = 0;
	right_wheel_ticks = 0;

	v_x = 0.0;
	v_y = 0.0;
	w_z = 0.0;

	x = 0.0;
	y = 0.0;
	th = 0.0;  

	dt = 0.0;
}

void Listener::init(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nh;

	subscriber = nh.subscribe(SUB_NAME, SUB_QUEUE_SIZE, &Listener::callback, this);
	publisher = nh.advertise<nav_msgs::Odometry>(PUB_NAME, PUB_QUEUE_SIZE);
}

void Listener::callback(const magabot::Status::ConstPtr& msg)
{
	right_wheel_ticks = msg->encoders.right_wheel;
	left_wheel_ticks = msg->encoders.left_wheel;
	dt = msg->timestamp.toSec() - dt;

	double delta_th = K_TH * double(right_wheel_ticks - left_wheel_ticks);
	double delta_x = K_XY * double(right_wheel_ticks + left_wheel_ticks) * cos(th);
	double delta_y = K_XY * double(right_wheel_ticks + left_wheel_ticks) * sin(th);

	x += delta_x;
	y += delta_y;
	th += delta_th;

	v_x = double(delta_x / dt);
	v_y = double(delta_y / dt);
	w_z = double(delta_th / dt);
	odom_quaternion = tf::createQuaternionMsgFromYaw(th);
}

void Listener::publish()
{
	time_stamp = ros::Time::now();

	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = ODOM_FRAME_ID;

	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.orientation = odom_quaternion;

	odom.child_frame_id = ODOM_CHILD_FRAME_ID;
	odom.twist.twist.linear.x = v_x;
	odom.twist.twist.linear.y = v_y;
	odom.twist.twist.angular.z = w_z;

	odom.pose.covariance[0] = 0.001;
	odom.pose.covariance[7] = 0.001;
	odom.pose.covariance[14] = 0.001;
	odom.pose.covariance[21] = 1000;
	odom.pose.covariance[28] = 1000;
	odom.pose.covariance[35] = 1000;

	odom.twist.covariance = odom.pose.covariance;

	publisher.publish(odom);
}

void Listener::run()
{
	ros::Rate loopRate(PUB_RATE);

	while (ros::ok()) {
		ros::spinOnce();
		publish();
		loopRate.sleep();
	}
}

int main(int argc, char **argv)
{
	Listener listener;
	listener.init(argc, argv);
	listener.run();

	return 0;
}