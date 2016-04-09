#include <iostream>
#include "signal.h"
#include <ros/ros.h>
#include "AutoNav.h"

boost::shared_ptr<AutoNav> an;

/**
 * Handles a given signal by landing the drone and exiting the program.
 * sig: Intercepted signal
 */
void handleTerm(int sig)
{
	std::cout << "SIGNAL: " << sig << " Landing! DO NOT FORCE TERMINATE!!!" << std::endl;
	an->land();
	exit(1);
}

int main(int argc, char** argv){

	ros::init(argc, argv, "quadroter_auto_nav");
	ros::NodeHandle nh;

	// Catch ABRT, TERM, and INT signals and handle them
	signal(SIGABRT,handleTerm);
    signal(SIGTERM,handleTerm);
    signal(SIGINT, handleTerm);

	an.reset(new AutoNav(nh));

	// Start the navigation process
	an->doNav();

}
