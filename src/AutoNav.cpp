#include "AutoNav.h"

nav_msgs::OccupancyGrid::ConstPtr map;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
	std::cout << "Got map";
	map = msg;
}

AutoNav::AutoNav(ros::NodeHandle &n)
{
	nh = n;
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	map_sub = nh.subscribe("map", MAP_QUEUE_SIZE, mapCallback);
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
	int i = 0;
	while(i < 2)
	{
		std::cout << "Up" << std::endl;
		sendMessage(0,0,0.5,0,0,0);
		std::cout << "Sleep" << std::endl;
		ros::Duration(2).sleep(); // sleep in seconds
		std::cout << "Move" << std::endl;
		sendMessage(0,-0.5,0,0,0,0);
		std::cout << "Sleep" << std::endl;
		ros::Duration(2).sleep(); // sleep in seconds
		i++;
	}
}