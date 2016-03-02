#ifndef AUTO_NAV
#define AUTO_NAV

//C++ Headers
#include <cstddef>
#include <iostream>
#include <unistd.h>

//ROS Headers
#include <ros/ros.h>

//ROS Topic Headers
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Range.h>

//TF Headers
#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>

#include "Problem.h"
#include "State.h"
#include <vector>

class AutoNav{

public:
	AutoNav();
	AutoNav(ros::NodeHandle &n);
	void doNav();
	void land();
	void moveTo(float x, float y, float z);
	void sendMessage(float linX, float linY, float linZ, float angX, float angY, float angZ);
	void addPoint(float x, float y, float z, visualization_msgs::Marker &points);
	visualization_msgs::Marker createPoints(float a, float r, float g, float b);
	void publishPoints(visualization_msgs::Marker &points);
	int getGridXPoint(float point);
	int getGridYPoint(float point);
	int getTransformXPoint(int gridPoint);
	int getTransformYPoint(int gridPoint);
private:
	//ROS Main Handler
	ros::NodeHandle nh;

	//ROS Publishers
	ros::Publisher cmd_vel_pub;	//used to publish movements
	ros::Publisher marker_pub;	//used to publish colors

	//ROS Subscribers
	ros::Subscriber map_sub;	//gets the current occupancy grid
	ros::Subscriber sonar_sub;	//gets the current sonar readings

	//TF
	tf::TransformListener listener;	//listens for location updates
	tf::StampedTransform transform;	//holds our last known location
};

#endif