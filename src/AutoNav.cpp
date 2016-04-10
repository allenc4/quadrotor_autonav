#include "AutoNav.h"
#include "CommonUtils.h"
#include <sys/time.h>

typedef unsigned long long timestamp_t;

/**
 * Returns the current time in microseconds
 */
static timestamp_t get_timestamp ()
{
  struct timeval now;
  gettimeofday (&now, NULL);
  return  now.tv_usec + (timestamp_t)now.tv_sec * 1000000;
}

nav_msgs::OccupancyGrid::ConstPtr map;
sensor_msgs::Range::ConstPtr height;
geometry_msgs::PointStamped goal;
bool hasPointGoal = false;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
	map = msg;
}

void sonarCallback(const sensor_msgs::Range::ConstPtr &msg)
{
	height = msg;
}

void goalCallback(const geometry_msgs::PointStamped &msg)
{
	goal = msg;
	hasPointGoal = true;
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
	goal_sub = nh.subscribe("/clicked_point", 1, goalCallback);

}

/*
 * Function takes in a given x and y coordinate value, calculates which way to turn, and turns
 * so the front of the drone is facing the point (rotates on the z-axis).
 * Note: This function will determine rotation speed (if needed), and sets the az velocity as necessary.
 * This function must be called within a loop, as this function does NOT wait until the drone has rotated to a particular point.
 *
 * x: X coordinate to point towards
 * y: Y coordinate to point towards
 * az: angular velocity value on the z axis
 * return: angle (in radians) that we need to rotate (sign of angle is used for rotational direction)
 */
double AutoNav::lookAt(int x, int y, float &az)
{
	// Get the current position of the drone
	int curX = CommonUtils::getGridXPoint(transform.getOrigin().x(), map);
	int curY = CommonUtils::getGridYPoint(transform.getOrigin().y(), map);

	// Normalizes the angle to be between 0 circle to + 2 * M_PI circle. Returns in radians.
	double angle = angles::normalize_angle_positive(PI + tf::getYaw(transform.getRotation()));

	// Get a point 10 units in the positive direction on the X axis from the current location
	int baseX = curX - 10;
	int baseY = curY;

	// Now, we have three points on a triangle to compute the angle. So get the distances
	double baseToCurDist = CommonUtils::getDistance(baseX, baseY, curX, curY);
	double curToNextDist = CommonUtils::getDistance(curX, curY, x, y);
	double baseToNextDist = CommonUtils::getDistance(baseX, baseY, x, y);

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
		az = -0.15;
	}
	else if (angleDif >= -0.44 && angleDif < 0)
	{
		az = 0.15;
	}
	else if (angleDif > 0.44)
	{
		az = -1.75;
	}
	else if (angleDif < -0.44)
	{
		az = 1.75;
	}

	// Add points to RVIZ so we can see which points are being used to calculate the angle of the triangle
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

/**
 * Send a geometry twist message (used to do the actual movement by ROS) using
 * the given linear and angular velocity parameters.
 */
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


/**
 * Main function of the AutoNav class that actually performs the navigation.
 * Here, we traverse the entire map until we have fully explored it.
 * We then wait for user input to go to a certain point.
 */
void AutoNav::doNav(){

	// Setup and initialize various parameters and flags
	float lx = 0.0, ly = 0.0f, lz = 0.0f;
	float ax = 0.0f, ay = 0.0f, az = 0.0f;

	int gridx, gridy;
	int currentIndex;
	std::vector<State> path;
	bool atHeight = false;
	int startPathSize = 0;
	bool lookingAtGoal = false;
	bool fullyExplored = false;
	float startingXPoint = 0.0;
	float startingYPoint = 0.0;

	// For ROS, everything (outside of setups and initializations) is done within this loop
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
				lz = -0.25;
			}
			else if(!atHeight && transform.getOrigin().z() < 1)
			{
				startingYPoint = oy;
				startingXPoint = ox;
				std::cout << "To Low Going Up..." << std::endl;
				//start with getting off the ground
				lz = 0.5;
				az = 0.75; // As we go up, turn so we can get lidar data and start building the map
			}
			else if(!atHeight && transform.getOrigin().z() >= 1)
			{
				std::cout << "At Height" << startingXPoint << " " << startingYPoint << std::endl;
				atHeight = true;
				lz = -0.25;
			}
			else if(path.size() == 0 && atHeight == true && !fullyExplored)
			{
				lookingAtGoal = false;
				std::cout << "Finding path..." << std::endl;
				gridx = CommonUtils::getGridXPoint(ox, map);
				gridy = CommonUtils::getGridYPoint(oy, map);

				//if we are looking for a path then we stop
				sendMessage(0,0,0,0,0,0);
				
				currentIndex = CommonUtils::getIndex(gridx,gridy, map);
				Problem p(nh, map);
				State startState(gridx, gridy, map->data[currentIndex]);

				// Get a path to the nearest goal
				path = p.search(startState);
				if(path.size() > 0){
					std::cout << "Got Path with size " << path.size() << " and Cost " << path.front().priority << " and Divided Cost " << path.front().priority/path.size() << std::endl;
					startPathSize = path.size();
				}else{
					// No more states to explore, so map is fully traversed and known.
					std::cout << "Couldn't Find a path to -1 fully explored!" << std::endl;
					std::cout << "Now you may specify a point to go to via RVIZ" << std::endl;
					fullyExplored = true;
				}
			}
			else if(path.size() == 0 && fullyExplored)
			{
				sendMessage(0,0,0,0,0,0);
				
				if(hasPointGoal)
				{
					// User selected a goal point in RVIZ
					std::cout << "Goal Selected (" << goal.point.x << ", " << goal.point.y << ")" << std::endl;
					hasPointGoal = false; 

					Problem p(nh, map);

					int startX = CommonUtils::getGridXPoint(ox, map);
					int startY = CommonUtils::getGridYPoint(oy, map);
					int startIndex = CommonUtils::getIndex(startX,startY, map);
					State startState(startX, startY, map->data[startIndex]);

					int goalX = CommonUtils::getGridXPoint(goal.point.x, map);
					int goalY = CommonUtils::getGridYPoint(goal.point.y, map);
					int goalIndex = CommonUtils::getIndex(goalX,goalY, map);
					State goalState(goalX, goalY, goalIndex);

					// Search for a path to the goal
					path = p.search(startState, goalState);
					if(path.size() == 0)
					{
						std::cout << "We were unable to find a path to (" << goal.point.x << ", " << goal.point.y << ")" << std::endl;
						std::cout << "Please try picking a new point" << std::endl;
					}
				}
			}
			else if(path.size() > 0 && !lookingAtGoal)
			{
				double angle = lookAt(path.front().x, path.front().y, az);
				if(angle <= 0.0872665)
				{
					lookingAtGoal = true;
					if(map->data[CommonUtils::getIndex(path.front().x, path.front().y, map)] != -1 && !fullyExplored)
					{
						std::cout << "Goal explored" << std::endl;
						path.clear();
					}
				}
			}
			else if(path.size() > 0)
			{
				// There is a path defined, so traverse it
				// std::cout << "Traversing Path" << std::endl;
				State nextPath = path.back();
				bool obstacle = nextPath.obstacle;

				int averageX = 0;
				int averageY = 0;
				int pointsToAverage = 4;
				int pointsAveraged = 1;

				// Average the next three points to attempt to get a smoother flight forward
				// std::cout << "Averaging path..." << std::endl;
				if(path.size() >= pointsToAverage && obstacle == false)
				{
					averageX += path.back().x;
					averageY += path.back().y;
					for(int i = path.size()-2; i > path.size()-1-pointsToAverage; i--)
					{
						if (path.at(i).obstacle) {
							// If there is an obstacle near a given point (within the allowed threshold),
							// do not average the points and just use the next point.
							obstacle = true;
							averageX = nextPath.x;
							averageY = nextPath.y;
							break;
						}
						averageX += path.at(i).x;
						averageY += path.at(i).y;
						pointsAveraged++;
					}
					if (!obstacle) {
						averageX /= pointsAveraged;
						averageY /= pointsAveraged;
					}
				}else
				{
					averageX = nextPath.x;
					averageY = nextPath.y;
				}

				// Place the point to face in RVIZ
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
				double angleDif = fabs(lookAt(averageX, averageY, az));
				if (angleDif <= 0.75)
				{
					// Angle is within ~45 degrees, so keep moving (speed based on angle difference)
					lx = 0.5 - angleDif;
					if (lx > 0.5) {
						lx -= (lx - 0.4);
					}
				}
				if(lx < 0 || lx > 0.75) lx = 0; //dont go backwards

				//if we get half way through the path lets recalculate
				if(path.size() <= startPathSize/2)
				{
					// std::cout << "Recalculating..." << std::endl;
					path.clear();
				}

				// Show the path in RVIZ
				for(std::vector<State>::iterator i = path.begin(); i != path.end(); ++i)
				{
					debug->addPoint(CommonUtils::getTransformXPoint(i->x, map), CommonUtils::getTransformYPoint(i->y, map), 0);
				}
			}
			lookatDebug->publishPoints();  // Place all lookatDebug points added in RVIZ UI
			averagedDebug->publishPoints();  // Place all averagedDebug points added in RVIZ UI
			debug->publishPoints();  // Place all debug points added in RVIZ UI
			sendMessage(lx, ly, lz, ax, ay, az);
		}catch(tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			continue;
		}
	}
}

/**
 * Slowly land the drone.
 */
void AutoNav::land()
{
	sendMessage(0,0,-0.25,0,0,0);
}
