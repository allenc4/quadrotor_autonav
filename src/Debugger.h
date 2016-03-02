#ifndef _DEBUGGER_
#define _DEBUGGER_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class Debugger{
public:
	Debugger(ros::NodeHandle &n);
	Debugger(ros::NodeHandle &n, float r, float g, float b);
	void addPoint(float x, float y, float z);
	void publishPoints();
	void removePoints();
private:
	void init(float r, float g, float b);
	visualization_msgs::Marker points;
	ros::Publisher marker_pub;
};

#endif