#include "ros/ros.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"

#define nodeName "robot_footprint"

#define pubName "footprint"
#define pubQueue 1000

int main(int argc, char **argv)
{
  ros::init(argc, argv, nodeName);
  ros::NodeHandle n;

  ros::Publisher publisher = n.advertise<geometry_msgs::Polygon>(pubName, pubQueue);

  ros::Rate loopRate(40);

  geometry_msgs::Polygon msg;

	msg.points.resize(5);

	msg.points[0].x = -0.1923; msg.points[0].y = -0.1855; msg.points[0].z = 0.0307;
	msg.points[1].x = 0; msg.points[1].y = -0.2255; msg.points[1].z = 0.0307;
	msg.points[2].x = 0.1923; msg.points[2].y = -0.1855; msg.points[2].z = 0.0307;
	msg.points[3].x = 0.1923; msg.points[3].y = 0.2255; msg.points[3].z = 0.0307;
	msg.points[4].x = -0.1923; msg.points[4].y = 0.2255; msg.points[4].z = 0.0307;

  while (ros::ok())
  {
    publisher.publish(msg);

    ros::spinOnce();

    loopRate.sleep();
  }

  return 0;
}
