#ifndef AUTO_NAV
#define AUTO_NAV

#define MAP_QUEUE_SIZE 10

#include <ros/ros.h>
#include <iostream>
#include <unistd.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>

class AutoNav{

public:
	AutoNav(ros::NodeHandle &n);
	void doNav();
	void sendMessage(float linX, float linY, float linZ, float angX, float angY, float angZ);
private:
	ros::NodeHandle nh;
	ros::Publisher cmd_vel_pub;
	ros::Subscriber map_sub;
};

#endif