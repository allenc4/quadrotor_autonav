#include "AutoNav.h"

nav_msgs::OccupancyGrid::ConstPtr map;
sensor_msgs::Range::ConstPtr height;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
	map = msg;
}

void sonarCallback(const sensor_msgs::Range::ConstPtr &msg)
{
	height = msg;
}

AutoNav::AutoNav(ros::NodeHandle &n)
{
	nh = n;
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

	map_sub = nh.subscribe("/map", 1, mapCallback); //topic, queuesize, callback
	sonar_sub = nh.subscribe("/sonar_height", 1, sonarCallback);
}

void AutoNav::moveTo(float x, float y, float z)
{
	
}

visualization_msgs::Marker AutoNav::createPoints(float a, float r, float g, float b){
	visualization_msgs::Marker points;
	points.header.frame_id = "/world";
	points.header.stamp = ros::Time::now();
	points.ns = "/quadcopter_points";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;

	points.id = 1;

	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.05;
	points.scale.y = 0.05;
	points.scale.z = 0.05;

	points.color.a = a;
	points.color.r = r;
	points.color.g = g;
	points.color.b = b;

	return points;
}

void AutoNav::addPoint(float x, float y, float z, visualization_msgs::Marker &points){
	geometry_msgs::Point p;
	p.x = x;
	p.y = y;
	p.z = z;

	points.points.push_back(p);
}

void AutoNav::publishPoints(visualization_msgs::Marker &points)
{
	marker_pub.publish(points);
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

int AutoNav::getGridXPoint(float point)
{
	return (int)((point - map->info.origin.position.x) / map->info.resolution);
}

int AutoNav::getGridYPoint(float point)
{
	return (int)((point - map->info.origin.position.x) / map->info.resolution);
}

int AutoNav::getTransformXPoint(int gridPoint)
{
	return (float)((gridPoint * map->info.resolution) + map->info.origin.position.x);
}

int AutoNav::getTransformYPoint(int gridPoint)
{
	return (float)((gridPoint * map->info.resolution) + map->info.origin.position.y);
}

void AutoNav::doNav(){

	float lx = 0.0, ly = 0.0f, lz = 0.0f;
	float ax = 0.0f, ay = 0.0f, az = 0.0f;

	int gridx, gridy;
	int currentIndex;
	visualization_msgs::Marker points = this->createPoints(1,1,0,0);
	while(nh.ok())
	{
		ros::spinOnce(); // needed to get subscribed messages

		lx = ly = lz = 0;
		ax = ay = az = 0;

		try{
			//gets the current transform from the map to the base_link
			listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			//gets us our current x,y coordinate in the occupancy grid
			gridx = this->getGridPoint(transform.getOrigin().x());
			gridy = this->getGridPoint(transform.getOrigin().y());
			
			visualization_msgs::Marker points = this->createPoints(1,1,0,0);
			this->addPoint(transform.getOrigin().x(), transform.getOrigin().y(), 0, points);
			this->publishPoints(points);

			//converts current x,y to single index
			currentIndex = (gridx+10) * (gridy+1) - 1;

			std::cout << "Attempting search..." << std::endl;
			Problem p(map);
			State startState(gridx, gridy, map->data[currentIndex]);
			std::vector<State> path = p.search(startState);
			std::cout << "Search finished with " << path.size() << " points." << std::endl;

			for(std::vector<State>::iterator i = path.begin(); i != path.end(); ++i)
			{
				std::cout << "(" << i->x << ", " << i->y << ")" << std::endl;
			}

			sendMessage(lx,ly,lz,ax,ay,az);
		}catch(tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			continue;
		}
	}
}

void AutoNav::land()
{
	sendMessage(0,0,-0.25,0,0,0);
}