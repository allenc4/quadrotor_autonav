#ifndef _COMMONUTILS_
#define _COMMONUTILS_

#include <iostream>
#include <nav_msgs/OccupancyGrid.h>

namespace CommonUtils{
	int getGridXPoint(float point, nav_msgs::OccupancyGrid::ConstPtr map);
	int getGridYPoint(float point, nav_msgs::OccupancyGrid::ConstPtr map);
	float getTransformXPoint(int gridPoint, nav_msgs::OccupancyGrid::ConstPtr map);
	float getTransformYPoint(int gridPoint, nav_msgs::OccupancyGrid::ConstPtr map);
	int getIndex(int gridx, int gridy, nav_msgs::OccupancyGrid::ConstPtr map);
	double getDistance(int x1, int y1, int x2, int y2);
}
#endif