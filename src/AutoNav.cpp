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
	map_sub = nh.subscribe("/map", 1, mapCallback); //topic, queuesize, callback
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

	float lx = 0.0, ly = 0.0f, lz = 0.0f;
	float ax = 0.0f, ay = 0.0f, az = 0.0f;

	int gridx, gridy;
	int currentIndex, north, south, east, west;

	while(nh.ok())
	{
		ros::spinOnce(); // needed to get subscribed messages
		tf::StampedTransform transform;

		lx = ly = lz = 0;
		ax = ay = az = 0;
		north = south = west = east = -1;

		try{
			//gets the current transform from the map to the base_link
			listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			//gets us our current x,y coordinate in the occupancy grid
			gridx = (unsigned int)((transform.getOrigin().x() - map->info.origin.position.x) / map->info.resolution);
			gridy = (unsigned int)((transform.getOrigin().y() - map->info.origin.position.y) / map->info.resolution);
			//converts current x,y to single index
			currentIndex = (gridx+1) * (gridy+1) - 1;

			//TODO: Get circle encompassing front/left/right of quadcoptor with values of occupancy if we are going to hit something stop

			//need to get these to not be under the quadrotor
			if(gridy > 0)
			{
				north = (gridx+1) * (gridy) -1;				//get the immediatly next north grid point
			}
			if(gridy < map->info.height - 1)
			{
				south = (gridx+1) * (gridy+2) -1;			//gets the immeditaly next south grid point
			}
			if(gridx < map->info.width)
			{
				east = (gridx+2) * (gridy+1) -1;			//gets the immeditaly next east grid point
			}
			if(gridx > 0)
			{
				west = (gridx) * (gridy+1) - 1;				//gets the immeditaly next west grid point
			}

			//print all the shits
			std::cout << "(" << transform.getOrigin().x() << ", " << transform.getOrigin().y() << ", " << transform.getOrigin().z() << ") (" 
			<< gridx << ", " << gridy << ") (" << currentIndex  << ", " << north << ", " << south << ", " << east << ", " << west << ")" << std::endl;


			//go up until we are one meter off the ground
			if(transform.getOrigin().z() < 1)
			{
				std::cout << "Up" << std::endl;
				lz = 0.5;
			}else if(map->data[currentIndex] > 0)
			{
				lx = 0;
				ly = 0;
				ax = 0.5;
			}else
			{
				ly = -0.5;
				// Get a list of obstacles around the UAV
				getSurroundingPoints(gridx, gridy, 15);
			}

			sendMessage(lx,ly,lz,ax,ay,az);
		}catch(tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			continue;
		}
	}
}

// Gets a list of occupancy grid indexes where there is an obstacle
// within a square around the UAV (boundary length determined by threshold)
void AutoNav::getSurroundingPoints(int centerX, int centerY, int threshold) {
	std::vector<int> occupiedIndexies;

	for (int i = centerX - threshold; i <= centerX + threshold; i++) {
		for (int j = centerY - threshold; j <= centerY + threshold; j++) {
			// Convert x and y into single index for occupancy grid
			int curIndex = (i * map->info.width) + j;
			if (map->data[curIndex] == POSITIVE_OBJECT_OCCUPIED) {
				occupiedIndexies.push_back(index);
			}
		}
	}
}

void AutoNav::land()
{
	sendMessage(0,0,-0.25,0,0,0);
}
