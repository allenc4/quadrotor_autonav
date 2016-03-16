#ifndef _DEBUGGER_
#define _DEBUGGER_

#include <ros/ros.h>
#include <string>
#include <visualization_msgs/Marker.h>

class Debugger{
public:
	Debugger(ros::NodeHandle &n, std::string ns);
	Debugger(ros::NodeHandle &n, std::string ns, float r, float g, float b);
	void addPoint(float x, float y, float z);
	void publishPoints();
	void removePoints();
	void turnOff();
	void turnOn();
private:
	void init(float r, float g, float b, std::string ns);
	visualization_msgs::Marker points;
	ros::Publisher marker_pub;
	bool on;
};

#endif