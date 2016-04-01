#include "CommonUtils.h"

int CommonUtils::getGridXPoint(float point, nav_msgs::OccupancyGrid::ConstPtr map)
{
	return (int)((point - map->info.origin.position.x) / map->info.resolution);
}


int CommonUtils::getGridYPoint(float point, nav_msgs::OccupancyGrid::ConstPtr map)
{
	return (int)((point - map->info.origin.position.x) / map->info.resolution);
}


float CommonUtils::getTransformXPoint(int gridPoint, nav_msgs::OccupancyGrid::ConstPtr map)
{
	return (float)((gridPoint * map->info.resolution) + map->info.origin.position.x);
}


float CommonUtils::getTransformYPoint(int gridPoint, nav_msgs::OccupancyGrid::ConstPtr map)
{
	return (float)((gridPoint * map->info.resolution) + map->info.origin.position.y);
}


int CommonUtils::getIndex(int gridx, int gridy, nav_msgs::OccupancyGrid::ConstPtr map)
{
	return gridx + gridy * map->info.width;
}

double CommonUtils::getDistance(int x1, int y1, int x2, int y2)
{
	return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}