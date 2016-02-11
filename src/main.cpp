#include <iostream>
#include "signal.h"
#include <ros/ros.h>
#include "AutoNav.h"

boost::shared_ptr<AutoNav> an;

void handleTerm(int sig)
{
	std::cout << "SIGNAL: " << sig << " Landing! DO NOT FORCE TERMINATE!!!" << std::endl;
	an->land();
	exit(1);
}

int main(int argc, char** argv){

	ros::init(argc, argv, "quadroter_auto_nav");
	ros::NodeHandle nh;

	signal(SIGABRT,handleTerm);
    signal(SIGTERM,handleTerm);
    signal(SIGINT, handleTerm);

	
	an.reset(new AutoNav(nh));

	an->doNav();

}