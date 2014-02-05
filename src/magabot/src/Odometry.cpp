#include <math.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "magabot/Status.h"

#define _USE_MATH_DEFINES

const char* NODE_NAME = "odometryCalculator";

const char* PUB_NAME = "odom";
const unsigned int PUB_QUEUE_SIZE = 1000;
const unsigned int PUB_RATE = 80;

const char* SUB_NAME = "/magabot/Status";
const unsigned int SUB_QUEUE_SIZE = 1000;

const char* ODOM_FRAME_ID = "odom";
const char* ODOM_CHILD_FRAME_ID = "base_footprint";

const float DISTANCE = 0.345;
const float RADIUS = 0.045;
const float TICKS_REVOLUTION = 3900;

const float K_TH = 2 * M_PI * RADIUS / (DISTANCE * TICKS_REVOLUTION);
const float K_XY = RADIUS * M_PI / TICKS_REVOLUTION;

ros::Subscriber sub;
ros::Publisher pub;

void callback(const magabot::Status::ConstPtr& msg)
{

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

	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = ODOM_FRAME_ID;

	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);

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

	pub.publish(odom);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);

	ros::NodeHandle nh;

	pub = nh.advertise<nav_msgs::Odometry>(PUB_NAME, PUB_QUEUE_SIZE);

	sub = nh.subscribe(SUB_NAME, SUB_QUEUE_SIZE, callback);

	ros::spin();

	return 0;
}
