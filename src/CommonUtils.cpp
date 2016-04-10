#include "CommonUtils.h"

/**
 * Get the x coordinate in relation to the occupancy grid, with the point based on the transformation x value
 *
 * point: X point retrieved from the stamped transform object
 * map: OccupancyGrid map instance
 * returns: X coordinate of the point in relation to the occupancy grid
 */
int CommonUtils::getGridXPoint(float point, nav_msgs::OccupancyGrid::ConstPtr map)
{
	return (int)((point - map->info.origin.position.x) / map->info.resolution);
}

/**
 * Get the y coordinate in relation to the occupancy grid, with the point based on the transformation y value
 *
 * point: Y point retrieved from the stamped transform object
 * map: OccupancyGrid map instance
 * returns: Y coordinate of the point in relation to the occupancy grid
 */
int CommonUtils::getGridYPoint(float point, nav_msgs::OccupancyGrid::ConstPtr map)
{
	return (int)((point - map->info.origin.position.x) / map->info.resolution);
}

/**
 * Convert the x coordinate point from the occupancy grid to the a transformation point
 *
 * gridPoint: X point of the occupancy grid
 * map: OccupancyGrid map instance
 * returns: X coordinate of the point in relation to the grid transformation
 */
float CommonUtils::getTransformXPoint(int gridPoint, nav_msgs::OccupancyGrid::ConstPtr map)
{
	return (float)((gridPoint * map->info.resolution) + map->info.origin.position.x);
}

/**
 * Convert the y coordinate point from the occupancy grid to the a transformation point
 *
 * gridPoint: Y point of the occupancy grid
 * map: OccupancyGrid map instance
 * returns: Y coordinate of the point in relation to the grid transformation
 */
float CommonUtils::getTransformYPoint(int gridPoint, nav_msgs::OccupancyGrid::ConstPtr map)
{
	return (float)((gridPoint * map->info.resolution) + map->info.origin.position.y);
}

/**
 * Returns the index of the occupancy grid in relation to a given map coordinate.
 * Note: to retrieve the map coordinates, use the getGridX() and getGridY() methods.
 *
 * gridx: X coordinate on the map
 * gridy: Y coordinate on the map
 * map: OccupancyGrid map instance
 * returns: The index of the (gridx,gridy) coordinate in the map array
 */
int CommonUtils::getIndex(int gridx, int gridy, nav_msgs::OccupancyGrid::ConstPtr map)
{
	return gridx + gridy * map->info.width;
}

/**
 * Returns the Euclidean distance of two points.
 *
 * x1: X coordinate of point 1
 * y1: Y coordinate of point 1
 * x2: X coordinate of point 2
 * y2: Y coordinate of point 2
 * returns: Euclidean (straight-line) distance between the two points
 */
double CommonUtils::getDistance(int x1, int y1, int x2, int y2)
{
	return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}
