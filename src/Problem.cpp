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
	this->debug2 = new Debugger(nh, "Goal_State", 0,0,1);
	this->map = map;
}

Problem::~Problem(){
	delete debug;
}

bool Problem::isGoalState(State state){
	return (state.value == -1);
}

//currently only did N,S,E,W can do NE,NW,SE,SW after
std::vector<State> Problem::getSuccessors(State state){
	std::vector<State> successors;
	int successorOffset = 1;
	if(state.y > successorOffset)
	{
		int northIndex = CommonUtils::getIndex(state.x, state.y+successorOffset, map);				//get the immediatly next north grid point
		if(map->data[northIndex] <= 0)
		{
			State north(state.x, state.y+1, map->data[northIndex]);
			north.cost = state.cost+1;
			successors.push_back(north);
		}
	}
	if(state.y < map->info.height - successorOffset)
	{
		int southIndex = CommonUtils::getIndex(state.x, state.y-successorOffset, map);			//gets the immeditaly next south grid point
		if(map->data[southIndex] <= 0)
		{
			State south(state.x, state.y-1, map->data[southIndex]);
			south.cost = state.cost+1;
			successors.push_back(south);
		}
	}
	if(state.x < map->info.width - successorOffset)
	{
		int eastIndex = CommonUtils::getIndex(state.x+successorOffset, state.y, map);			//gets the immeditaly next east grid point
		if(map->data[eastIndex] <= 0)
		{
			State east(state.x+1, state.y, map->data[eastIndex]);
			east.cost = state.cost+1;
			successors.push_back(east);
		}
	}
	if(state.x > successorOffset)
	{
		int westIndex = CommonUtils::getIndex(state.x-successorOffset, state.y, map);				//gets the immeditaly next west grid point
		if(map->data[westIndex] <= 0)
		{
			State west(state.x-1, state.y, map->data[westIndex]);
			west.cost = state.cost+1;
			successors.push_back(west);
		}
	}
	return successors;
}

int Problem::heuristic(State state){
	return 0; //trivial heuristic
}

bool Problem::checkStateForObstacle(State state){
	return false;
}

std::vector<State> Problem::search(State startState){

	std::cout << "Attempting search..." << std::endl;
	std::set<State, SetCompare> closedList;  //used the SetCompare to compare the x,y 

	std::priority_queue<State> frontier;

	frontier.push(startState);

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

		// for(std::set<State>::iterator i = closedList.begin(); i != closedList.end(); ++i)
		// {
		// 	std::cout << " (" << i->x << ", " << i->y << ", " << i->priority << "),";
		// }
		// std::cout << std::endl;

		//std::cout << "Processing state at (" << state.x << ", " << state.y << ") Priority: " << state.priority << " Value: " << state.value << " Path Size: " << state.path.size();
		if(this->isGoalState(*state))
		{
			//std::cout << "Found Path " << state.path.size() << std::endl;
			debug->publishPoints();
			std::vector<State> path;

			std::cout << "Found goal (" << state->x << ", " << state->y << ") Parent: " << state->parent << std::endl;

			State * temp = state;

			while(temp->parent != NULL)
			{
				std::cout << "Adding (" << temp->x << ", " << temp->y << ") to path. Parent: " << temp->parent << " Current: " << &temp << std::endl;
				path.push_back(*temp);
				temp = temp->parent;
			}

			return path;
		}else if(closedList.find(*state) == closedList.end()){
			closedList.insert(*state); 
			std::vector<State> successors = this->getSuccessors(*state);
			for(int i = 0; i < successors.size(); i++)
			{
				debug->addPoint(CommonUtils::getTransformXPoint(state->x, map), CommonUtils::getTransformYPoint(state->y, map), 0);
				State successor = successors[i];
				successor.priority = successor.cost + this->heuristic(successor);
				successor.parent = state;
				//std::cout << "\tSuccessor: (" << successor.x << ", " << successor.y << ") Priority: " << successor.priority << " Parent: " << successor.parent << std::endl;
				frontier.push(successor);
			}
		}
	}

	debug->publishPoints();
	
	std::vector<State> path;
	return path;//no path 
}
