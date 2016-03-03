#include "State.h"

State::State(int x, int y, float value){
	this->x = x;
	this->y = y;
	this->value = value;
	priority = 0;
	parent = NULL;
}

State::~State(){
	//delete parent;
}