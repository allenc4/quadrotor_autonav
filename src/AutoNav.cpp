#include "AutoNav.h"

nav_msgs::OccupancyGrid::ConstPtr map;
sensor_msgs::Range::ConstPtr height;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
	std::cout << "Got map" << std::endl;
	map = msg;
}

void sonarCallback(const sensor_msgs::Range::ConstPtr &msg)
{
	std::cout << "Got height: " << msg << std::endl;
	height = msg;
}

AutoNav::AutoNav(ros::NodeHandle &n)
{
	nh = n;
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	map_sub = nh.subscribe("/map", 1, mapCallback);
	sonar_sub = nh.subscribe("/sonar_height", 1, sonarCallback);
}

void AutoNav::sendMessage(float linX, float linY, float linZ, float angX, float angY, float angZ)
{
	geometry_msgs::Twist cmd;
	cmd.linear.x = linX;
	cmd.linear.y = linY;
	cmd.linear.z = linZ;
	cmd.angular.x = angX;
	cmd.angular.y = angY;
	cmd.angular.z = angZ;

	cmd_vel_pub.publish(cmd);
}

void AutoNav::doNav(){
	
	while(nh.ok())
	{
		if(height == NULL || height->range < 1)
		{
			std::cout << "Up" << std::endl;
			sendMessage(0,0,0.5,0,0,0);
			std::cout << "Sleep" << std::endl;
			ros::Duration(2).sleep(); // sleep in seconds
		}else
		{
			std::cout << "Move" << std::endl;
			sendMessage(0,-0.5,0,0,0,0);
			std::cout << "Sleep" << std::endl;
			ros::Duration(2).sleep(); // sleep in seconds
			std::cout << "Move" << std::endl;
			sendMessage(0,0.5,0,0,0,0);
			std::cout << "Sleep" << std::endl;
			ros::Duration(2).sleep(); // sleep in seconds
		}

    	ros::spinOnce(); // needed to get subscribed messages
	}
}

void AutoNav::land()
{
	sendMessage(0,0,-0.5,0,0,0);
}