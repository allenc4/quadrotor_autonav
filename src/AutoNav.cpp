#include "AutoNav.h"
#include "CommonUtils.h"

nav_msgs::OccupancyGrid::ConstPtr map;
sensor_msgs::Range::ConstPtr height;
geometry_msgs::PoseStamped::ConstPtr pose;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
	map = msg;
}

void sonarCallback(const sensor_msgs::Range::ConstPtr &msg)
{
	height = msg;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	pose = msg;
}

AutoNav::AutoNav(ros::NodeHandle &n)
{
	nh = n;
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	debug = new Debugger(nh, "AutoNav");
	map_sub = nh.subscribe("/map", 1, mapCallback); //topic, queue size, callback
	sonar_sub = nh.subscribe("/sonar_height", 1, sonarCallback);
	pose_sub = nh.subscribe("/slam_out_pose", 1, poseCallback);

}

void AutoNav::moveTo(float x, float y, float z)
{
	
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
	int currentIndex;
	while(nh.ok())
	{
		ros::spinOnce(); // needed to get subscribed messages

		lx = ly = lz = 0;
		ax = ay = az = 0;

		try{
			//gets the current transform from the map to the base_link
			listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			//gets us our current x,y coordinate in the occupancy grid
			float oy = transform.getOrigin().y();
			float ox = transform.getOrigin().x();
			gridx = CommonUtils::getGridXPoint(ox, map);
			gridy = CommonUtils::getGridYPoint(oy, map);
			
			currentIndex = CommonUtils::getIndex(gridx,gridy, map);
	 		std::cout << "Attempting search..." << std::endl;
			Problem p(nh, map);
			State startState(gridx, gridy, map->data[currentIndex]);
			std::vector<State> path = p.search(startState);
			std::cout << "Search finished with " << path.size() << " points." << std::endl;

			for(std::vector<State>::iterator i = path.begin(); i != path.end(); ++i)
			{
				std::cout << "(" << i->x << ", " << i->y << ")" << std::endl;
				debug->addPoint(CommonUtils::getTransformXPoint(i->x, map), CommonUtils::getTransformYPoint(i->y, map), 0);
			}

			
			
			debug->publishPoints();		

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
			if (map->data[curIndex] == this->MAP_POSITIVE_OBJECT_OCCUPIED) {
				occupiedIndexies.push_back(curIndex);
			}
		}
	}
}

void AutoNav::land()
{
	sendMessage(0,0,-0.25,0,0,0);
}
