#include "AutoNav.h"

AutoNav::AutoNav(ros::NodeHandle &nh)
{
	nh_ = nh;
	cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

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
	
	while(true)
	{
		sendMessage(0,0,0.5,0,0,0);
	}

}