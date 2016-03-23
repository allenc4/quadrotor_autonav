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
	lookatDebug = new Debugger(nh, "lookat");
	map_sub = nh.subscribe("/map", 1, mapCallback); //topic, queue size, callback
	sonar_sub = nh.subscribe("/sonar_height", 1, sonarCallback);
	pose_sub = nh.subscribe("/slam_out_pose", 1, poseCallback);
}

//void AutoNav::moveTo(tf::StampedTransform pose, float x, float y, float & lx, float & ly)
void AutoNav::moveTo(float x, float y, float & lx, float & ly)
{
	
}

//void AutoNav::lookAt(tf::StampedTransform pose, float x, float y, float & ax, float & ay)
void AutoNav::lookAt(int x, int y, float &az)
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

	double angleDif = angle - baseToNextAngle;

	bool switchDir = false;
	if (angleDif > PI) {
		switchDir = true;
	}

	std::cout << "Quadrotor Angle: " << (int) angles::to_degrees(angle) <<
				"   baseToNext: " << (int) angles::to_degrees(baseToNextAngle) <<
				"   compensation: " << (int) angles::to_degrees(angleDif) <<
				"   switchDir: " << switchDir << std::endl;

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

	if (switchDir) {
		az = -az;
	}

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

//	// Fix the angle
//	double angleDeg = angles::to_degrees(angle);
//	if (angleDeg > 90 && angleDeg < 180) {
//		angleDeg -= 180;
//	} else if (angleDeg >= 180 && angleDeg < 270) {
//		angleDeg -= 180;
//	} else if (angleDeg >= 270) {
//		angleDeg -= 360;
//	}
//	// Get a point 25 units on the current facing line of sight
//	int newX = 25 * cos(angles::from_degrees(angleDeg));
//	int newY = 25 * sin(angles::from_degrees(angleDeg));


//	if (angles::to_degrees(angle) > 90 && angles::to_degrees(angle) <= 270) {
//		newX += curX;
//		newY += curY;
//	} else {
//		newX = curX - newX;
//		newY = curY - newY;
//	}


	// Now, we have three points on a triangle to compute the angle. So get the distances
//	double newToCurDist = getDistance(newX, newY, curX, curY);
//	double curToNextDist = getDistance(curX, curY, x, y);
//	double newToNextDist = getDistance(newX, newY, x, y);
//
//	// We need to get the angle opposite from the new coordinate to the next coordinate (newToNextDist),
//	// because that is the angle we are going to be turning to look at the next position.
//
//	// Use the cosine rule since sine will only give us acute angles
//	double newToNextAngle = acos(
//			(pow(newToCurDist, 2) + pow(curToNextDist, 2) - pow(newToNextDist, 2)) / (2 * newToCurDist * curToNextDist)  // Get cos(newToNextDist)
//			);
//
//
////	angle = angles::normalize_angle_positive(angle + newToNextAngle);
//	angle -= newToNextAngle;
//
//	std::cout << "Quadrotor Angle: " << angles::to_degrees(angles::normalize_angle_positive(PI + tf::getYaw(transform.getRotation()))) <<
//			"   newToNext: " << angles::to_degrees(newToNextAngle) <<
//			std::endl;
//
//	// Check if we have to change direction
//	bool switchDir = false;
//	if (prevAngle == -1) {
//		prevAngle = newToNextAngle;
//	} else if (prevAngle != -1 && prevAngle > newToNextAngle) {
//		switchDir = true;
//		prevAngle = newToNextAngle;
//	}
//
//	double angleDif = angles::normalize_angle(tf::getYaw(transform.getRotation()) - angle - PI);
////	std::cout << "Angle compensation: " << angles::to_degrees(angleDif) << std::endl;
//
//////	// ~5 degrees either way so stop moving
//	if(angleDif <= 0.0872665)
//	{
//		az = 0;
//	}
//	else if (angleDif <= 0.44 && angleDif > 0)
//	{
//		if (az >= 0) {
//			az = 0.05;
//		} else {
//			az = -0.05;
//		}
//	}
//	else if (angleDif > 0.44)  // ~25 degrees
//	{
//		if (az >= 0) {
//			az = 0.75;
//		} else {
//			az = -0.75;
//		}
//	}
//
//	if (az != 0 && switchDir) {
//		az = -1 * az;
//	}
//	az = 0.15;
//
//	lookatDebug->addPoint(CommonUtils::getTransformXPoint(newX, map),
//			CommonUtils::getTransformYPoint(newY, map),
//			0);


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
				atHeight = false;
				lz = 0.25;
			}
			else if(!atHeight && transform.getOrigin().z() < 1)
			{
				//start with getting off the ground
				lz = 0.5;
			}
			else if(!atHeight && transform.getOrigin().z() >= 1)
			{
				atHeight = true;
				lz = -0.25;
			}
			else if(path.size() == 0 && atHeight == true)
			{
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
				int averageX = 0;
				int averageY = 0;
				int pointsToAverage = 4;

				if(path.size() >= pointsToAverage)
				{
					for(int i = path.size()-1; i > path.size()-1-pointsToAverage; i--)
					{
						averageX += path.at(i).x;
						averageY += path.at(i).y;
					}
					averageX /= pointsToAverage;
					averageY /= pointsToAverage;
				}else
				{
					averageX = path.front().x;
					averageY = path.front().y;
				}

				float nextX = CommonUtils::getTransformXPoint(averageX, map);
				float nextY = CommonUtils::getTransformYPoint(averageY, map);

				float thresholdX = abs(averageX - CommonUtils::getGridXPoint(transform.getOrigin().x(), map));
				float thresholdY = abs(averageY - CommonUtils::getGridYPoint(transform.getOrigin().y(), map));

				//if we get close to the point then remove it from the path
				if(thresholdX <= 2 && thresholdY <= 2)
				{
					std::cout << "Got to point" << std::endl;
					path.pop_back();
				}

				//if for some reason re magically become really far away
				//recalculate the path.  we probably hit a wall or something.
				if(thresholdX >= 20 || thresholdY >= 20){
					std::cout << "To Far away" << std::endl;
					path.clear();
					lx = 0;
					az = 0;
				}else
				{					
					//look at the averaged point
					lookAt(averageX, averageY, az);					
					lx = 0.5-az;
					if(lx < 0 || lx > 0.5) lx = 0; //dont go backwards 
				}

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
//void AutoNav::getSurroundingPoints(int centerX, int centerY, int threshold) {
//	std::vector<int> occupiedIndexies;
//
//	for (int i = centerX - threshold; i <= centerX + threshold; i++) {
//		for (int j = centerY - threshold; j <= centerY + threshold; j++) {
//			// Convert x and y into single index for occupancy grid
//			int curIndex = (i * map->info.width) + j;
//			if (map->data[curIndex] == this->MAP_POSITIVE_OBJECT_OCCUPIED) {
//				occupiedIndexies.push_back(curIndex);
//			}
//		}
//	}
//}

void AutoNav::land()
{
	sendMessage(0,0,-0.25,0,0,0);
}
