#ifndef _STATE_
#define _STATE_

#include <vector>
#include <iostream>
#include <cstddef>

/**
 * Small class to hold various data about a state. Used during problem search.
 */
class State{
public:
	State(){};
	State(int x, int y, float value);
	~State();
	int x;
	int y;
	float value;
	float cost;
	int priority;
	bool obstacle;
	State * parent;
	bool operator< (const State& rhs) const {
		return this->priority > rhs.priority; //have to switch the signs because
											//priority queue sorts largest to smallest
											//we want it in reverse
	}
private:
};

#endif
