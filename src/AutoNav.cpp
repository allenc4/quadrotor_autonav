#include "AutoNav.h"
#include "CommonUtils.h"
#include <sys/time.h>

typedef unsigned long long timestamp_t;

static timestamp_t get_timestamp ()
{
  struct timeval now;
  gettimeofday (&now, NULL);
  return  now.tv_usec + (timestamp_t)now.tv_sec * 1000000;
}

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
	lookatDebug = new Debugger(nh, "lookat", 0,0,255);
	averagedDebug = new Debugger(nh, "averaged", 0,255,255);
	map_sub = nh.subscribe("/map", 1, mapCallback); //topic, queue size, callback
	sonar_sub = nh.subscribe("/sonar_height", 1, sonarCallback);
}

//void AutoNav::lookAt(tf::StampedTransform pose, float x, float y, float & ax, float & ay)
double AutoNav::lookAt(int x, int y, float &az)
{
	int curX = CommonUtils::getGridXPoint(transform.getOrigin().x(), map);
	int curY = CommonUtils::getGridYPoint(transform.getOrigin().y(), map);

	// Normalizes the angle to be between 0 circle to + 2 * M_PI circle. Returns in radians.
	double angle = angles::normalize_angle_positive(PI + tf::getYaw(transform.getRotation()));

	// Get a point 10 units in the positive direction on the X axis from the current location
	int baseX = curX - 10;
	int baseY = curY;


	// Now, we have three points on a triangle to compute the angle. So get the distances
	double baseToCurDist = getDistance(baseX, baseY, curX, curY);
	double curToNextDist = getDistance(curX, curY, x, y);
	double baseToNextDist = getDistance(baseX, baseY, x, y);

	// We need to get the angle opposite from the new coordinate to the next coordinate (baseToNextDist),
	// because that is the angle we are going to be turning to look at the next position.

	// Use the cosine rule since sine will only give us acute angles
	double baseToNextAngle = acos(
			(pow(baseToCurDist, 2) + pow(curToNextDist, 2) - pow(baseToNextDist, 2)) / (2 * baseToCurDist * curToNextDist)  // Get cos(baseToNextDist)
			);

	if (y > baseY) {
		baseToNextAngle = TWO_PI - baseToNextAngle;
	}

	double angleDif = angles::normalize_angle(angle - baseToNextAngle);

//	bool switchDir = false;
//	if (angleDif > PI || angleDif < -PI) {
//		angleDif = angles::normalize_angle(baseToNextAngle - angle);
//	}

//	std::cout << "Quadrotor Angle: " << (int) angles::to_degrees(angle) <<
//				"   baseToNext: " << (int) angles::to_degrees(baseToNextAngle) <<
//				"   compensation: " << (int) angles::to_degrees(angleDif) << std::endl;
//				"   switchDir: " << switchDir << std::endl;

	// ~5 degrees either way so stop moving
	if (angleDif >= -0.0872665 && angleDif <= 0.0872665)
	{
		az = 0;
	}
	else if (angleDif > 0 && angleDif <= 0.44)  // ~ 25 degrees
	{
//		az = -abs(angleDif-.15);
		az = -0.15;
	}
	else if (angleDif >= -0.44 && angleDif < 0)
	{
//		az = abs(angleDif + .15);
		az = 0.15;
	}
	else if (angleDif > 0.44)
	{
//		az = -abs(angleDif + .5);
		az = -1.75;
	}
	else if (angleDif < -0.44)
	{
//		az = abs(angleDif - .5);
		az = 1.75;
	}

//	if (switchDir) {
//		az = -az;
//	}

//	az = 0.25;

	lookatDebug->addPoint(CommonUtils::getTransformXPoint(curX, map),
				CommonUtils::getTransformYPoint(curY, map),
				0);
	lookatDebug->addPoint(CommonUtils::getTransformXPoint(x, map),
				CommonUtils::getTransformYPoint(y, map),
				0);
	lookatDebug->addPoint(CommonUtils::getTransformXPoint(baseX, map),
					CommonUtils::getTransformYPoint(baseY, map),
					0);

	return angleDif;

}

double AutoNav::getDistance(int x1, int y1, int x2, int y2)
{
	return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
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
	int startPathSize = 0;

	while(nh.ok())
	{
		ros::spinOnce(); // needed to get subscribed messages

		debug->removePoints();
		lookatDebug->removePoints();
		averagedDebug->removePoints();

		lx = ly = lz = 0;
		ax = ay = az = 0;

		try
		{
			//gets the current transform from the map to the base_link
			listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			//gets us our current x,y coordinate in the occupancy grid
			float oy = transform.getOrigin().y();
			float ox = transform.getOrigin().x();

			if(transform.getOrigin().z() >= 1.5)
			{
				std::cout << "To High going down" << std::endl;
				atHeight = false;
				lz = 0.25;
			}
			else if(!atHeight && transform.getOrigin().z() < 1)
			{
				std::cout << "To Low Going Up..." << std::endl;
				//start with getting off the ground
				lz = 0.5;
			}
			else if(!atHeight && transform.getOrigin().z() >= 1)
			{
				std::cout << "At Height" << std::endl;
				atHeight = true;
				lz = -0.25;
			}
			else if(path.size() == 0 && atHeight == true)
			{
				std::cout << "Finding path..." << std::endl;
				gridx = CommonUtils::getGridXPoint(ox, map);
				gridy = CommonUtils::getGridYPoint(oy, map);
				
				currentIndex = CommonUtils::getIndex(gridx,gridy, map);
				Problem p(nh, map);
				State startState(gridx, gridy, map->data[currentIndex]);

				path = p.search(startState);
				std::cout << "Got Path with size " << path.size() << " and Cost " << path.front().priority << std::endl;
				startPathSize = path.size();
			}
			else if(path.size() > 0)
			{
				std::cout << "Traversing Path" << std::endl;
				State nextPath = path.back();
				bool obstacle = nextPath.obstacle;

				int averageX = 0;
				int averageY = 0;
				int pointsToAverage = 4;

				std::cout << "Averageing path..." << std::endl;
				if(path.size() >= pointsToAverage && obstacle == false)
				{
					for(int i = path.size()-1; i > path.size()-1-pointsToAverage; i--)
					{
						if (path.at(i).obstacle) {
							obstacle = true;
							averageX = nextPath.x;
							averageY = nextPath.y;
							break;
						}
						averageX += path.at(i).x;
						averageY += path.at(i).y;
					}
					if (!obstacle) {
						averageX /= pointsToAverage;
						averageY /= pointsToAverage;
					}
				}else
				{
					averageX = nextPath.x;
					averageY = nextPath.y;
				}

				averagedDebug->addPoint(CommonUtils::getTransformXPoint(averageX, map), CommonUtils::getTransformYPoint(averageY, map), 0);


				float thresholdX = abs(averageX - CommonUtils::getGridXPoint(transform.getOrigin().x(), map));
				float thresholdY = abs(averageY - CommonUtils::getGridYPoint(transform.getOrigin().y(), map));

				//if we get close to the point then remove it from the path
				if(thresholdX <= 2 && thresholdY <= 2)
				{
					// std::cout << "Got to point" << std::endl;
					path.pop_back();
				}

				//look at the next (or average) point
				//if no obstacles, move if within angle range
				//if obstacle, move ONLY if angle range is within ~5 degrees so we dont hit anything
				double angleDif = fabs(lookAt(averageX, averageY, az));
				if (obstacle == true && angleDif <= 0.0872665) {
					// There is an obstacle, so only move if the angle difference is minuscule
					lx = 0.15;
					// std::cout << "Obstacle detected. Small angle difference. Proceed with caution." << std::endl;
				}
				else if (obstacle == false && angleDif <= 0.75) {
					// Angle is within ~45 degrees, so keep moving if not within any obstacles
					lx = 0.75 - angleDif;
					if (lx > 0.5) {
						lx -= (lx - 0.4);
					}
					// std::cout << "No obstacles. Moving..." << lx << std::endl;
				}
				if(lx < 0 || lx > 0.75) lx = 0; //dont go backwards

				//if we get half way through the path lets recalculate
				if(path.size() <= startPathSize/2)
				{
					path.clear();
				}

				for(std::vector<State>::iterator i = path.begin(); i != path.end(); ++i)
				{
					debug->addPoint(CommonUtils::getTransformXPoint(i->x, map), CommonUtils::getTransformYPoint(i->y, map), 0);
				}
			}
			debug->publishPoints();
			lookatDebug->publishPoints();
			averagedDebug->publishPoints();
			sendMessage(lx, ly, lz, ax, ay, az);
		}catch(tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			continue;
		}
	}
}

// Gets a list of occupancy grid indexes where there is an obstacle
// within a square around the UAV (boundary length determined by threshold)
//bool AutoNav::checkForObstacles(int curX, int curY) {
//	int threshold = 8;
//
//	for(int y = -threshold; y < threshold; y++)
//	{
//		for(int x = -threshold; x < threshold; x++)
//		{
//			if(map->data[CommonUtils::getIndex(curX + x, curY + y, map)] > 0)
//			{
//				return true;
//			}
//		}
//	}
//	return false;
//}

void AutoNav::land()
{
	sendMessage(0,0,-0.25,0,0,0);
}
