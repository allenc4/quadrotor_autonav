#ifndef _PROBLEM_
#define _PROBLEM_

#include <cstddef>
#include <vector>
#include <set>
#include <queue>
#include <string>
#include "State.h"
#include <nav_msgs/OccupancyGrid.h>
#include "CommonUtils.h"
#include "Debugger.h"
#include <cmath>
#include <algorithm>

/**
 * Class defines the basic search algorithms used as well as other problems that
 * will define functions for goal testing, successors, and the heuristic to use.
 */
class Problem{
public:
	Problem(ros::NodeHandle &nh, nav_msgs::OccupancyGrid::ConstPtr map);
	~Problem();
	bool isGoalState(State state);
	std::vector<State> getSuccessors(State state);
	int heuristic(State& state);
	std::vector<State> search(State state);
	std::vector<State> search(State startState, State goalState);
	int checkStateForObstacle(State& state);

private:
	std::vector<State> goalStates;
	nav_msgs::OccupancyGrid::ConstPtr map;
	Debugger * debug;
	int successorOffset;

	static const int MAP_UNEXPLORED = -1;
	static const int MAP_POSITIVE_OBJECT_OCCUPIED = 100;
	static const int MAP_POSITIVE_OBJECT_UNOCCUPIED = 0;
	static const int OBSTACLE_THRESHOLD = 9;
};


#endif
