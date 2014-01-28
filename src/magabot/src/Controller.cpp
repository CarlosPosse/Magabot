#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "magabot/Actuation.h"

#define _USE_MATH_DEFINES

#define NODE_NAME "Controller"

#define PUB_NAME "/magabot/Actuation"
#define PUB_QUEUE_SIZE 1000
#define PUB_RATE 100

#define SUB_NAME "cmd_vel"
#define SUB_QUEUE_SIZE 1000

#define DISTANCE 0.345
#define RADIUS 0.045
#define TICKS_PER_REV 3900

#define K_MOTOR_RATIO TICKS_PER_REV * 0.00332 / (2 * M_PI * RADIUS)

class Listener
{

private:
	int right_motor_speed;
	int left_motor_speed;

	magabot::Actuation actuation_msg;

	double v_x;
	double v_y;
	double w_z;

	ros::Subscriber subscriber;
	ros::Publisher publisher;

	void publish();
	void callback(const geometry_msgs::Twist::ConstPtr& msg);

public:
	Listener();

	void init(int argc, char **argv);
	void run();
	
};

Listener::Listener()
{
	right_motor_speed = 0.0;
	left_motor_speed = 0.0;

	v_x = 0.0;
	v_y = 0.0;
	w_z = 0.0;
}

void Listener::init(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nh;

	subscriber = nh.subscribe(SUB_NAME, SUB_QUEUE_SIZE, &Listener::callback, this);
	publisher = nh.advertise<magabot::Actuation>(PUB_NAME, PUB_QUEUE_SIZE);
}

void Listener::callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	v_x = msg->linear.x;
	v_y = msg->linear.y;
	w_z = msg->angular.z;

	right_motor_speed = (int)((v_x - (w_z * DISTANCE / 2)) * K_MOTOR_RATIO);
	left_motor_speed = (int)((v_x + (w_z * DISTANCE / 2)) * K_MOTOR_RATIO);
}

void Listener::publish()
{
	actuation_msg.set_velocity = true;

	actuation_msg.velocity.right_wheel = right_motor_speed;
	actuation_msg.velocity.left_wheel = left_motor_speed;

	publisher.publish(actuation_msg);
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