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

class AutoNav{

public:
	AutoNav();
	AutoNav(ros::NodeHandle &n);
	void doNav();
	void land();
	void sendMessage(float linX, float linY, float linZ, float angX, float angY, float angZ);
private:
	ros::NodeHandle nh;
	ros::Publisher cmd_vel_pub;
	ros::Subscriber map_sub;
	ros::Subscriber sonar_sub;
	tf::TransformListener listener;
};

#endif