#include "Problem.h"

Problem::Problem(nav_msgs::OccupancyGrid::ConstPtr map){
	this->map = map;
}

Problem::~Problem(){

}

bool Problem::isGoalState(State state){
	return (state.value == -1);
}

//currently only did N,S,E,W can do NE,NW,SE,SW after
std::vector<State> Problem::getSuccessors(State state){
	std::vector<State> successors;
	if(state.y > 0)
	{
		int northIndex = (state.x+1) * (state.y) -1;				//get the immediatly next north grid point
		if(map->data[northIndex] <= 0)
		{
			State north(state.x, state.y+1, map->data[northIndex]);
			north.cost = state.cost+1;
			successors.push_back(north);
		}
	}
	if(state.y < map->info.height - 1)
	{
		int southIndex = (state.x+1) * (state.y+2) -1;			//gets the immeditaly next south grid point
		if(map->data[southIndex] <= 0)
		{
			State south(state.x, state.y-1, map->data[southIndex]);
			south.cost = state.cost+1;
			successors.push_back(south);
		}
	}
	if(state.x < map->info.width)
	{
		int eastIndex = (state.x+2) * (state.y+1) -1;			//gets the immeditaly next east grid point
		if(map->data[eastIndex] <= 0)
		{
			State east(state.x+1, state.y, map->data[eastIndex]);
			east.cost = state.cost+1;
			successors.push_back(east);
		}
	}
	if(state.x > 0)
	{
		int westIndex = (state.x) * (state.y+1) - 1;				//gets the immeditaly next west grid point
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

	std::set<State> closedList;
	std::priority_queue<State> frontier;

	frontier.push(startState);

	while(!frontier.empty())
	{
		State state = frontier.top();
		frontier.pop();
		std::cout << "Processing state at (" << state.x << ", " << state.y << ") Prio: " << state.priority << " Vaule: " << state.value << " Path Size: " << state.path.size() << std::endl;

		if(this->isGoalState(state))
		{
			std::cout << "Found Path " << state.path.size() << std::endl;
			return state.path;
		}else if(closedList.find(state) == closedList.end()){
			std::cout << "State not in closed list" << std::endl;
			closedList.insert(state); 
			std::vector<State> successors = this->getSuccessors(state);
			std::cout << "Got successors " << successors.size() << std::endl;
			for(int i = 0; i < successors.size(); i++)
			{
				State successor = successors[i];
				successor.priority = successor.cost + this->heuristic(successor);
				std::cout << "Successor: (" << successor.x << ", " << successor.y << ") Prio: " << successor.priority << " Path Size: " << successor.path.size() << std::endl;
				successor.path = state.path;
				successor.path.push_back(state);
				frontier.push(successor);
			}
		}
	}
	std::vector<State> path;
	return path;//no path 
}