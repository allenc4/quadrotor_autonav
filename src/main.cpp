#include "AutoNav.h"
#include <ros/ros.h>

int main(int argc, char** argv){

	ros::init(argc, argv, "quadroter_auto_nav");

	ros::NodeHandle nh;

	AutoNav an(nh);
	an.doNav();

}