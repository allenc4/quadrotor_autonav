#include "Problem.h"

struct SetCompare{
	bool operator() (const State& lhs, const State& rhs) const{
        return ((lhs.x != rhs.x) || (lhs.y != rhs.y));
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
	if(state.y > 0)
	{
		int northIndex = CommonUtils::getIndex(state.x, state.y+1, map);				//get the immediatly next north grid point
		if(map->data[northIndex] <= 0)
		{
			State north(state.x, state.y+1, map->data[northIndex]);
			north.cost = state.cost+1;
			successors.push_back(north);
		}
	}
	if(state.y < map->info.height - 1)
	{
		int southIndex = CommonUtils::getIndex(state.x, state.y-1, map);			//gets the immeditaly next south grid point
		if(map->data[southIndex] <= 0)
		{
			State south(state.x, state.y-1, map->data[southIndex]);
			south.cost = state.cost+1;
			successors.push_back(south);
		}
	}
	if(state.x < map->info.width)
	{
		int eastIndex = CommonUtils::getIndex(state.x+1, state.y, map);			//gets the immeditaly next east grid point
		if(map->data[eastIndex] <= 0)
		{
			State east(state.x+1, state.y, map->data[eastIndex]);
			east.cost = state.cost+1;
			successors.push_back(east);
		}
	}
	if(state.x > 0)
	{
		int westIndex = CommonUtils::getIndex(state.x-1, state.y, map);				//gets the immeditaly next west grid point
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


//NOTE This may be act weird for the following reasons:
// A set uses the < operator when inserting (I dont really get this as why doesn't it use the =)
//The Priority queue also uses the < operator to do its push.  The set needs to be by location the 
//priority queue is by cost+heuristic so it may act weird. 

//Second reason is im storing the path in the state object
//this may cause i problem where the objects are different in the set
//and therefore are never in the closedList
std::vector<State> Problem::search(State startState){

	std::set<State, SetCompare> closedList;
	std::priority_queue<State> frontier;

	frontier.push(startState);

	debug->removePoints();

	while(!frontier.empty())
	{
		State state = frontier.top(); //views top node
		frontier.pop(); //removes top node

		for(std::set<State>::iterator i = closedList.begin(); i != closedList.end(); ++i)
		{
			std::cout << " (" << i->x << ", " << i->y << ", " << i->priority << "),";
		}
		std::cout << std::endl;

		std::cout << "Processing state at (" << state.x << ", " << state.y << ") Priority: " << state.priority << " Value: " << state.value << " Path Size: " << state.path.size();
		if(this->isGoalState(state))
		{
			debug2->removePoints();
			debug2->addPoint(CommonUtils::getTransformXPoint(state.x, map), CommonUtils::getTransformYPoint(state.y, map), 0);
			debug2->publishPoints();
			std::cout << "Found Path " << state.path.size() << std::endl;
			return state.path;
		}else if(closedList.find(state) == closedList.end()){
			std::cout << " OPEN" << std::endl;
			closedList.insert(state); 
			std::vector<State> successors = this->getSuccessors(state);
			for(int i = 0; i < successors.size(); i++)
			{
				debug->addPoint(CommonUtils::getTransformXPoint(state.x, map), CommonUtils::getTransformYPoint(state.y, map), 0);
				State successor = successors[i];
				successor.priority = successor.cost + this->heuristic(successor);
				std::cout << "\tSuccessor: (" << successor.x << ", " << successor.y << ") Priority: " << successor.priority << " Path Size: " << successor.path.size() << std::endl;
				successor.path = state.path;
				successor.path.push_back(state);
				frontier.push(successor);
				debug->addPoint(CommonUtils::getTransformXPoint(successor.x, map), CommonUtils::getTransformYPoint(successor.y, map), 0);
			}
		}else
		{
			std::cout << " CLOSED" << std::endl;
		}
		debug->publishPoints();

	}
	std::vector<State> path;
	return path;//no path 
}