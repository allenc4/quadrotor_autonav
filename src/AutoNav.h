#ifndef AUTO_NAV
#define AUTO_NAV

//C++ Headers
#include <cstddef>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <cmath>

//ROS Headers
#include <ros/ros.h>

//ROS Topic Headers
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Range.h>

//TF Headers
#include <tf/transform_listener.h>

#include "Problem.h"
#include "State.h"
#include "Debugger.h"
#include <vector>
#include <angles/angles.h>

class AutoNav{

public:
	AutoNav();
	AutoNav(ros::NodeHandle &n);
	void doNav();
	void land();
//	void moveTo(tf::StampedTransform pose, float x, float y, float & lx, float & ly);
//	void lookAt(tf::StampedTransform pose, float x, float y, float & ax, float & ay);
	double lookAt(int x, int y, float &az);
	void sendMessage(float linX, float linY, float linZ, float angX, float angY, float angZ);
//	bool checkForObstacles(int curX, int curY);
private:
	//ROS Main Handler
	ros::NodeHandle nh;

	//ROS Publishers
	ros::Publisher cmd_vel_pub;	//used to publish movements

	//ROS Subscribers
	ros::Subscriber map_sub;	//gets the current occupancy grid
	ros::Subscriber sonar_sub;	//gets the current sonar readings
	ros::Subscriber goal_sub;	//gets the user designated goal

	//TF
	tf::TransformListener listener;	//listens for location updates
	tf::StampedTransform transform;	//holds our last known location

	Debugger * debug;
	Debugger * lookatDebug;
	Debugger * averagedDebug;

	static const int MAP_UNEXPLORED = -1;
	static const int MAP_POSITIVE_OBJECT_OCCUPIED = 100;
	static const int MAP_POSITIVE_OBJECT_UNOCCUPIED = 0;

	static const double PI = 3.14159265359;
	static const double TWO_PI = 6.28318530718;
	static const double PI_2 = 1.57079632679489661923;

};

#endif
