#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "magabot/Actuation.h"

#define _USE_MATH_DEFINES

const char* NODE_NAME = "Controller";

const char* PUB_NAME = "/magabot/Actuation";
const unsigned int PUB_QUEUE_SIZE = 1000;

const char* SUB_NAME = "cmd_vel";
const unsigned int SUB_QUEUE_SIZE = 1000;

const float DISTANCE = 0.345;
const float RADIUS = 0.045;
const float TICKS_PER_REV = 3900;
const float K_MOTOR_RATIO = TICKS_PER_REV * 0.00332 / (2 * M_PI * RADIUS);

ros::Subscriber sub;
ros::Publisher pub;

void callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	magabot::Actuation actuation_msg;

	double v_x = msg->linear.x;
	double v_y = msg->linear.y;
	double w_z = msg->angular.z;

	int right_motor_speed = (int)((v_x - (w_z * DISTANCE / 2)) * K_MOTOR_RATIO);
	int left_motor_speed = (int)((v_x + (w_z * DISTANCE / 2)) * K_MOTOR_RATIO);

	actuation_msg.set_velocity = true;

	actuation_msg.velocity.right_wheel = right_motor_speed;
	actuation_msg.velocity.left_wheel = left_motor_speed;

	pub.publish(actuation_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);

	ros::NodeHandle nh;

	pub = nh.advertise<magabot::Actuation>(PUB_NAME, PUB_QUEUE_SIZE);

	sub = nh.subscribe(SUB_NAME, SUB_QUEUE_SIZE, callback);

	ros::spin();

	return 0;
}