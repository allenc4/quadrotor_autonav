#include "Debugger.h"

Debugger::Debugger(ros::NodeHandle &n){
	this->marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
	this->init(1,0,0);
}

Debugger::Debugger(ros::NodeHandle &n, float r, float g, float b){
	this->marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
	this->init(r,g,b);
}

void Debugger::init(float r, float g, float b){
	points.header.frame_id = "/world";
	points.header.stamp = ros::Time::now();
	points.ns = "/quadcopter_points";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;

	points.id = 1;

	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.05;
	points.scale.y = 0.05;
	points.scale.z = 0.05;
	// points.scale.x = 10;
	// points.scale.y = 10;
	// points.scale.z = 10;



	points.color.a = 1;
	points.color.r = r;
	points.color.g = g;
	points.color.b = b;
}

void Debugger::addPoint(float x, float y, float z){
	geometry_msgs::Point p;
	p.x = x;
	p.y = y;
	p.z = z;

	points.points.push_back(p);
}

void Debugger::publishPoints()
{
	marker_pub.publish(points);
}

void Debugger::removePoints(){
	points.points.clear();
}