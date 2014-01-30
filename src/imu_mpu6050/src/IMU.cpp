#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "imu_mpu6050/Inertial.h"

const char* FRAME_ID = "imu";
const char* NODE_NAME = "IMU";
const char* PUB_NAME = "imu";
const char* SUB_NAME = "inertial";

const unsigned int PUB_QUEUE_SIZE = 1000;
const unsigned int SUB_QUEUE_SIZE = 1000;

const float GRAVITY = 9.80665;

ros::Subscriber sub;
ros::Publisher pub;

unsigned int seqID = 0;

void callback(const imu_mpu6050::Inertial::ConstPtr& msg)
{
	sensor_msgs::Imu imu_msg;

	imu_msg.header.seq = seqID;
	imu_msg.header.stamp = ros::Time::now();
	imu_msg.header.frame_id = FRAME_ID;

	imu_msg.orientation.w = msg->orientation.w;
	imu_msg.orientation.x = msg->orientation.x;
	imu_msg.orientation.y = msg->orientation.y;
	imu_msg.orientation.z = msg->orientation.z;

	imu_msg.orientation_covariance[0] = imu_msg.orientation_covariance[4] = imu_msg.orientation_covariance[8] = 0.00001;

	//imu_msg.angular_velocity.x = msg->angular_velocity.x / 131; // e converter para rad/s
	//imu_msg.angular_velocity.y = msg->angular_velocity.y / 131; //	"			"
	//imu_msg.angular_velocity.z = msg->angular_velocity.z / 131; //	"			"

	//imu_msg.angular_velocity_covariance[0] = imu_msg.angular_velocity_covariance[4] = imu_msg.angular_velocity_covariance[8] = 0.00001;

	imu_msg.angular_velocity_covariance[0] = -1;

	imu_msg.linear_acceleration.x = (msg->linear_acceleration.x / 16384) * GRAVITY;
	imu_msg.linear_acceleration.y = (msg->linear_acceleration.y / 16384) * GRAVITY;
	imu_msg.linear_acceleration.z = (msg->linear_acceleration.z / 16384) * GRAVITY;

	imu_msg.linear_acceleration_covariance[0] = imu_msg.linear_acceleration_covariance[4] = imu_msg.linear_acceleration_covariance[8] = 0.00001;

	pub.publish(imu_msg);

	seqID++;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);

	ros::NodeHandle nh;

	pub = nh.advertise<sensor_msgs::Imu>(PUB_NAME, PUB_QUEUE_SIZE);

	sub = nh.subscribe(SUB_NAME, SUB_QUEUE_SIZE, callback);

	ros::spin();

	return 0;
}