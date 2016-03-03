#include "AutoNav.h"
#include "CommonUtils.h"

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
	debug = new Debugger(nh, "AutoNav");
	map_sub = nh.subscribe("/map", 1, mapCallback); //topic, queuesize, callback
	sonar_sub = nh.subscribe("/sonar_height", 1, sonarCallback);
}

void AutoNav::moveTo(tf::StampedTransform pose, float x, float y, float & lx, float & ly)
{
	
}

void AutoNav::lookAt(tf::StampedTransform pose, float x, float y, float & ax, float & ay){
	
	ax = 0;//we never want to rotate on the x axis :P

	double angle = atan2(y-pose.getOrigin().y(), x-pose.getOrigin().x());
	if(angle > -5 && angle < 5)
	{
		ay = 0;
	}
	else if(angle < 0)
	{}
	
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
	std::vector<State> path;
	bool atHeight = false;
	while(nh.ok())
	{
		ros::spinOnce(); // needed to get subscribed messages

		debug->removePoints();

		lx = ly = lz = 0;
		ax = ay = az = 0;

		try{
			//gets the current transform from the map to the base_link
			listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			//gets us our current x,y coordinate in the occupancy grid
			float oy = transform.getOrigin().y();
			float ox = transform.getOrigin().x();

			if(!atHeight && transform.getOrigin().z() >= 1)
			{
				atHeight = true;
			}
			else if(transform.getOrigin().z() < 1)
			{
				//start with getting off the ground
				ax = 0.9;
				az = 0.9;
				ay = 0.9;
				lz = 0.25;
			}else if(path.size() == 0 && atHeight == true)
			{
				gridx = CommonUtils::getGridXPoint(ox, map);
				gridy = CommonUtils::getGridYPoint(oy, map);
				
				currentIndex = CommonUtils::getIndex(gridx,gridy, map);
				Problem p(nh, map);
				State startState(gridx, gridy, map->data[currentIndex]);
				path = p.search(startState);
				std::cout << "Got Path with size " << path.size() << std::endl;
			}else if(path.size() > 0)
			{
				State s = path.back();

				float nextX = CommonUtils::getTransformXPoint(s.x, map);
				float nextY = CommonUtils::getTransformXPoint(s.y, map);
				for(std::vector<State>::iterator i = path.begin(); i != path.end(); ++i)
				{
					debug->addPoint(CommonUtils::getTransformXPoint(i->x, map), CommonUtils::getTransformYPoint(i->y, map), 0);
				}
			}
			debug->publishPoints();		
			sendMessage(lx, ly, lz, ax, ay, az);
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