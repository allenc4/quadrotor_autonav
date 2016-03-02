#ifndef _PROBLEM_
#define _PROBLEM_

#include <vector>
#include <set>
#include <queue>
#include "State.h"
#include <nav_msgs/OccupancyGrid.h>
//Class Defines the Basic Search Algorithms 
//and an extendedable interface for other roblems that will define
//functions for goal testing, successors, and the heuristic to use

class Problem{
public:
	Problem(nav_msgs::OccupancyGrid::ConstPtr map);
	~Problem();
	bool isGoalState(State state);
	std::vector<State> getSuccessors(State state);
	int heuristic(State state);
	std::vector<State> search(State state);
	bool checkStateForObstacle(State state);
private:
	std::vector<State> goalStates;
	nav_msgs::OccupancyGrid::ConstPtr map;
};


#endif