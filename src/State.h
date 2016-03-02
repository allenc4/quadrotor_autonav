#ifndef _STATE_
#define _STATE_

#include <vector>
#include <iostream>

class State{
public:
	State(int x, int y, float value);
	int x;
	int y;
	float value;
	float cost;
	int priority;
	std::vector<State> path;
	bool operator< (const State& rhs) const {
		return this->priority > rhs.priority; //have to switch the signs because
											//priority queue sorts largest to smallest
											//we want it in reverse
	}
private:
};

#endif