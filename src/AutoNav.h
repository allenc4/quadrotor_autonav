#ifndef AUTO_NAV
#define AUTO_NAV

//C++ Headers
#include <cstddef>
#include <iostream>
#include <unistd.h>
#include <vector>

//ROS Headers
#include <ros/ros.h>

//ROS Topic Headers
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Range.h>

//TF Headers
#include <tf/transform_listener.h>

<<<<<<< HEAD
#define MAP_UNEXPLORED = -1
#define MAP_POSITIVE_OBJECT_OCCUPIED = 100
#define MAP_POSITIVE_OBJECT_UNOCCUPIED = 0
=======
#include "Problem.h"
#include "State.h"
#include "Debugger.h"
#include <vector>
>>>>>>> 84b6e0043058d8b37e55546301430f1b5d586588

class AutoNav{

public:
	AutoNav();
	AutoNav(ros::NodeHandle &n);
	void doNav();
	void land();
	void moveTo(float x, float y, float z);
	void sendMessage(float linX, float linY, float linZ, float angX, float angY, float angZ);
	void getSurroundingPoints(int centerX, int centerY, int threshold);
private:
	//ROS Main Handler
	ros::NodeHandle nh;

	//ROS Publishers
	ros::Publisher cmd_vel_pub;	//used to publish movements

	//ROS Subscribers
	ros::Subscriber map_sub;	//gets the current occupancy grid
	ros::Subscriber sonar_sub;	//gets the current sonar readings

	//TF
	tf::TransformListener listener;	//listens for location updates
	tf::StampedTransform transform;	//holds our last known location

	Debugger * debug;
};

#endif
