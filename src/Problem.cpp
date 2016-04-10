#include "Problem.h"

struct SetCompare{
	bool operator() (const State& lhs, const State& rhs) const{
		if(lhs.x == rhs.x)
		{
			return lhs.y < rhs.y;
		}
        return lhs.x < rhs.x;
    }
};

Problem::Problem(ros::NodeHandle &nh, nav_msgs::OccupancyGrid::ConstPtr map){
	this->debug = new Debugger(nh, "States_Expanded", 0,1,0);
	// this->debug->turnOff();
	this->map = map;
	this->successorOffset = 4;
}

Problem::~Problem(){
	delete debug;
}

/**
 * Checks to see if a particular state is a goal or not
 *
 * state: State instance to check
 * return: true if state is a goal; false otherwise
 */
bool Problem::isGoalState(State state){
	if(this->goalStates.size() > 0)
	{
		for(int i = 0; i < this->goalStates.size(); i++)
		{
			// If there is a goal within the successorOffset threshold, return true.
			// Use of this threshold is necessary because the state x and y values may not be exact
			if(CommonUtils::getDistance(this->goalStates.at(i).x, this->goalStates.at(i).y, state.x, state.y) <= this->successorOffset)
			{
				return true;
			}
		}
		return false;
	}

	// If there is no goal state defined, the map is not fully explored.
	// If the value of the state is unknown (MAP_UNEXPLORED), use this as a goal so we can explore it.
	return (state.value == Problem::MAP_UNEXPLORED);
}

/**
 * Retrieves a set of possible states reachable from the current state
 * (passed as a parameter) within one action.
 * This successor function evaluates 8 different states from the current state (N,S,E,W,NE,NW,SE,SW).
 *
 * state: Current state in which to generate successor states from
 * returns: Set of states reachable from the current state each with a total cost
 */
std::vector<State> Problem::getSuccessors(State state){
	std::vector<State> successors;

	if(state.y > this->successorOffset)
	{
		// Get the immediately next north grid point
		int northIndex = CommonUtils::getIndex(state.x, state.y+this->successorOffset, map);
		if(map->data[northIndex] <= 0)
		{
			State north(state.x, state.y+this->successorOffset, map->data[northIndex]);
			north.cost = state.cost+1;
			successors.push_back(north);
		}
	}
	if(state.y < map->info.height - this->successorOffset)
	{
		// Get the immediately next south grid point
		int southIndex = CommonUtils::getIndex(state.x, state.y-this->successorOffset, map);
		if(map->data[southIndex] <= 0)
		{
			State south(state.x, state.y-this->successorOffset, map->data[southIndex]);
			south.cost = state.cost+1;
			successors.push_back(south);
		}
	}
	if(state.x < map->info.width - this->successorOffset)
	{
		// Get the immediately next east grid point
		int eastIndex = CommonUtils::getIndex(state.x+this->successorOffset, state.y, map);
		if(map->data[eastIndex] <= 0)
		{
			State east(state.x+this->successorOffset, state.y, map->data[eastIndex]);
			east.cost = state.cost+1;
			successors.push_back(east);
		}
	}
	if(state.x > this->successorOffset)
	{
		// Get the immediately next west grid point
		int westIndex = CommonUtils::getIndex(state.x-this->successorOffset, state.y, map);
		if(map->data[westIndex] <= 0)
		{
			State west(state.x-this->successorOffset, state.y, map->data[westIndex]);
			west.cost = state.cost+1;
			successors.push_back(west);
		}
	}
	if(state.x > this->successorOffset && state.y > this->successorOffset)
	{
		// Get the immediately next north west grid point
		int northWestIndex = CommonUtils::getIndex(state.x-this->successorOffset, state.y+this->successorOffset, map);
		if(map->data[northWestIndex] <= 0)
		{
			State northWest(state.x-this->successorOffset, state.y+this->successorOffset, map->data[northWestIndex]);
			northWest.cost = state.cost+0.4 + 1;
			successors.push_back(northWest);
		}
	}
	if(state.x > this->successorOffset && state.y < map->info.height - this->successorOffset)
	{
		// Get the immediately next south west grid point
		int southWestIndex = CommonUtils::getIndex(state.x-this->successorOffset, state.y-this->successorOffset, map);
		if(map->data[southWestIndex] <= 0)
		{
			State southWest(state.x-this->successorOffset, state.y-this->successorOffset, map->data[southWestIndex]);
			southWest.cost = state.cost+0.4 + 1;
			successors.push_back(southWest);
		}
	}
	if(state.x < map->info.width - this->successorOffset && state.y > this->successorOffset)
	{
		// Get the immediately next north east grid point
		int northEastIndex = CommonUtils::getIndex(state.x+this->successorOffset, state.y+this->successorOffset, map);
		if(map->data[northEastIndex] <= 0)
		{
			State northEast(state.x+this->successorOffset, state.y+this->successorOffset, map->data[northEastIndex]);
			northEast.cost = state.cost+0.4 + 1;
			successors.push_back(northEast);
		}
	}
	if(state.x < map->info.width - this->successorOffset && state.y < map->info.height - this->successorOffset)
	{
		// Get the immediately next south east grid point
		int southEastIndex = CommonUtils::getIndex(state.x+this->successorOffset, state.y-this->successorOffset, map);
		if(map->data[southEastIndex] <= 0)
		{
			State southEast(state.x+this->successorOffset, state.y-this->successorOffset, map->data[southEastIndex]);
			southEast.cost = state.cost+0.4 + 1;
			successors.push_back(southEast);
		}
	}
	return successors;
}

/**
 * Return the estimate of how close the input state is (passed as a parameter) from a goal state.
 *
 * state: Input state. This heuristic will estimate the cost to the goal starting from this state.
 * returns: An estimated cost to reach the nearest goal from the input state
 */
int Problem::heuristic(State& state){
	int score = 0;

	// If the state is or is near an obstacle, add that estimated cost
	score += this->checkStateForObstacle(state);

	// If there is a goal state(s), use Euclidean distance to estimate
	// the cost to get from the current state to the goal
	if(this->goalStates.size() > 0)
	{
		int smallestDistance = 1000000;
		for(int i = 0; i < goalStates.size(); i++)
		{
			smallestDistance = std::min((int)CommonUtils::getDistance(state.x, state.y, this->goalStates.at(i).x, this->goalStates.at(i).y), smallestDistance);
		}
		score += smallestDistance;
	}
	return score;
}

/**
 * Checks if a given state contains an obstacle in or near that state (based on OBSTACLE_THRESHOLD).
 * If there is an obstacle, a number ranging from 0-1000000 will be returned based on how far away
 * the obstacle is from the state (the higher the number, the closer the obstacle is to the state).
 * Also, the obstacle flag for the state will be set if an obstacle is detected within the threshold.
 *
 * state: State in which the obstacle check will be performed on.
 * returns: Value ranging from 0 to 1000000 (0 meaning no obstacle, 1000000 meaning obstacle in state)
 */
int Problem::checkStateForObstacle(State& state){
	int score = 0;

	for(int y = -Problem::OBSTACLE_THRESHOLD; y < Problem::OBSTACLE_THRESHOLD; y++)
	{
		for(int x = -Problem::OBSTACLE_THRESHOLD; x < Problem::OBSTACLE_THRESHOLD; x++)
		{
			if(this->map->data[CommonUtils::getIndex(state.x+x, state.y+y, this->map)] > 0)
			{
				state.obstacle = true;
				score += 1000000/(std::max(abs(y), abs(x))+1);
			}
		}
	}
	return score;
}

/**
 * Sets a goal for the problem and searches for the optimal path to that goal.
 * When a goal state is given, A* search is performed to calculate the optimal path.
 *
 * startState: State to start from
 * goalState: Goal state to end at
 * returns: Set of vectors containing states from the start to the goal (optimal path).
 */
std::vector<State> Problem::search(State startState, State goalState){
	this->goalStates.clear();
	this->goalStates.push_back(goalState);
	return this->search(startState);
}

/**
 * Given a start state, searches for a solution to the problem.
 * If a goal was not set before calling this function, Uniform Cost Search will be performed
 * to find the optimal path to an unknown location.
 * If a goal state is known, use the search(startState, goalState) method.
 *
 * startState: State to start from
 * returns: Set of vectors containing states from the start to an unknown location (optimal path).
 */
std::vector<State> Problem::search(State startState){

	std::cout << "Attempting search..." << std::endl;
	std::set<State, SetCompare> closedList;  //used the SetCompare to compare the x,y 

	std::priority_queue<State> frontier;

	frontier.push(startState);  // Add the initial start state to the fringe

	debug->removePoints();

	while(!frontier.empty())
	{
		State s = frontier.top(); //views top node
		State * state = new State();
		state->x = s.x;
		state->y = s.y;
		state->cost = s.cost;
		state->parent = s.parent;
		state->value = s.value;
		state->priority = s.priority;
		frontier.pop(); //removes top node
	
		// std::cout << "Processing state at (" << state->x << ", " << state->y << ") Priority: " << state->priority << " Value: " << state->value << std::endl;
		if(this->isGoalState(*state))
		{
			// A goal was found, so return the path to get to the goal
			std::vector<State> path;

			// std::cout << "Found goal (" << state->x << ", " << state->y << ") Parent: " << state->parent << std::endl;

			State * temp = state;

			if(temp->parent == NULL)
			{
				this->goalStates.clear();
				path.push_back(*temp);
				return path;
			}

			while(temp->parent != NULL)
			{
				// std::cout << "Adding (" << temp->x << ", " << temp->y << ") to path. COST: " << temp->priority << std::endl;
				path.push_back(*temp);
				temp = temp->parent;
			}


			return path;
		}
		else if(closedList.find(*state) == closedList.end())
		{
			// Add the state to the closed set since this is the first time visiting it
			closedList.insert(*state); 
			std::vector<State> successors = this->getSuccessors(*state);  // Get list of successors
			debug->addPoint(CommonUtils::getTransformXPoint(state->x, map), CommonUtils::getTransformYPoint(state->y, map), 0);

			// Loop through the successors and choose their priority based on their successor cost plus the heuristic value
			for(int i = 0; i < successors.size(); i++)
			{
				State successor = successors[i];
				int heuristic = this->heuristic(successor);
				if(heuristic > 100000)
				{
					continue;
				}
				successor.priority = successor.cost + heuristic;

				successor.parent = state;
				// std::cout << "\tSuccessor: (" << successor.x << ", " << successor.y << ") Priority: " << successor.priority << " Parent: " << successor.parent << std::endl;
				frontier.push(successor);
			}
		}

		debug->publishPoints();
	}

	this->goalStates.clear();
	std::vector<State> path;
	return path;  // No path to goal found
}
