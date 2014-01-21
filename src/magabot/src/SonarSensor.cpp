#include "ros/ros.h"
#include "magabot/msgSensor.h"
#include <sensor_msgs/LaserScan.h>

#define nodeName "converterUS"

#define subName "rangeSonar"
#define subQueue 1000

#define pubName "scanSonar"
#define pubQueue 1000
#define frameId "frameSonar"

#define PI 3.14159
#define DegToRad PI/180

#define maxReadings 171 // Maximum number of readings 170+1
#define angleMin 0 // (º)
#define angleMax 180 // (º)
#define timeIncrement 0.80 // (s)
#define rangeMin 0.0 // (m)
#define rangeMax 1.0 // (m) 

class Listener{
	private:
		int currRead; // Current reading
		float distance; // Distance read
		float *data; // Buffer of distances read
		bool endSweep; // Bool to decide if a full sweep has been done

		// Message to publish
		sensor_msgs::LaserScan scanSonar;

		// Topic variables
		ros::Subscriber subscriber;
		ros::Publisher publisher;

		// Topic functions
		void addValue();
		void callback(const magabot::msgSensor::ConstPtr& msg);
		void publishMessage();
		
	public:
		Listener();
		~Listener();
		
		void init(int argc, char **argv);	
		void run();
};

Listener::Listener(){
	dadosD = (float*) malloc(numReadings * sizeof(float));
}

Listener::~Listener(){
	free(dadosD);
}

void Listener::init(int argc, char **argv){	
	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;

	subscriber = n.subscribe(subName, subQueue, &Listener::callback, this);
	publisher = n.advertise<sensor_msgs::LaserScan>(subPubName, pubQueue);

	scanSonar.header.frame_id = frameId;
	scanSonar.angle_min = angleMin * DegToRad;
	scanSonar.angle_max = angleMax * DegToRad;
	scanSonar.angle_increment = ((angleMax - angleMin)*DegToRad) / (numReadings-1);
	scanSonar.time_increment = timeIncrement;
	scanSonar.range_min = rangeMin;
	scanSonar.range_max = rangeMax;
}

void Listener::callback(const magabot::msgSensor::ConstPtr& msg){
	currRead = msg->currRead;
	distance = (msg->distance)/100;
	addValue();	
}

void Listener::addValue(){ 
	data[numReadings] = distance;

	if (currRead = maxReadings){
		endSweep = true;
	} else{
		endSweep = false;
	}
}

void Listener::publishMessage(){  
	ros::Time scan_time = ros::Time::now();

	//populate the LaserScan message  
	scanSonar.header.stamp = scan_time;
	
	scanSonar.ranges.resize(numReadings);
	scanSonar.intensities.resize(numReadings);

	for(unsigned int i = 0; i < numReadings; ++i){
		scanSonar.ranges[i] = data[i];
		scanSonar.intensities[i] = 0;
	}

	publisher.publish(scanSonar);
}

void Listener::run(){
	ros::Rate loopRate(100);

  while (ros::ok()){    
  	ros::spinOnce();

		if(endSweep = true){
  		publishMessage();
		}

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
