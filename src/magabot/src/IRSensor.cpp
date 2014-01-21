#include "ros/ros.h"
#include "magabot/msgSensor.h"
#include <sensor_msgs/LaserScan.h>

#define nodeName "ConverterIR"

#define subName "arduinoIR"
#define subQueue 1000

#define pubName "scanIR"
#define pubQueue 1000
#define frameId "frameIR"

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
		sensor_msgs::LaserScan scanIR;

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
	data = (float*) malloc(numReadings * sizeof(float));
}

Listener::~Listener(){
	free(data);
}

void Listener::init(int argc, char **argv){	
	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;

	subscriber = n.subscribe(subName, subQueue, &Listener::callback, this);
	publisher = n.advertise<sensor_msgs::LaserScan>(subPubName, pubQueue);

	scanIR.header.frame_id = frameId;
	scanIR.angle_min = angleMin * DegToRad;
	scanIR.angle_max = angleMax * DegToRad;
	scanIR.angle_increment = ((angleMax - angleMin)*DegToRad) / (numReadings-1);
	scanIR.time_increment = timeIncrement;
	scanIR.range_min = rangeMin;
	scanIR.range_max = rangeMax;
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
	scanIR.header.stamp = scan_time;
	
	scanIR.ranges.resize(numReadings);
	scanIR.intensities.resize(numReadings);

	for(unsigned int i = 0; i < numReadings; ++i){
		scanIR.ranges[i] = data[i];
		scanIR.intensities[i] = 0;
	}

	publisher.publish(scanIR);
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

