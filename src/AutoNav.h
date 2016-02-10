#ifndef AUTO_NAV
#define AUTO_NAV

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class AutoNav{

public:
	AutoNav(ros::NodeHandle &nh);
	void doNav();
	void sendMessage(float linX, float linY, float linZ, float angX, float angY, float angZ);
private:
	ros::NodeHandle nh_;
	ros::Publisher cmd_vel_pub;

};

#endif