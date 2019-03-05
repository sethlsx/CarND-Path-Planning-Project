#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
//#include "cost.h"
//#include <limits.h>
//#include <trajectory_generate.h>

using std::string;
using std::vector;

int SAFE_RADIUS = 10;
// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, double x, double y, double d, double s, double v, double vx, double vy, double yaw) 
{
  this->lane = lane;
  this->x = x;
  this->y = y;
  this->d = d;
  this->s = s;
  this->vx = vx;
  this->vy = vy;
  this->v = v;
  this->yaw = yaw;
  state = "KL";
  safe_radius = SAFE_RADIUS;
  //max_acceleration = -1;
}

Vehicle::~Vehicle() {}

vector<string> Vehicle::successor_states() 
{
  // Provides the possible next states given the current state for the FSM 
  //   discussed in the course, with the exception that lane changes happen 
  //   instantaneously, so LCL and LCR can only transition back to KL.
  vector<string> states;
  string state = this->state;
  if(state.compare("KL") == 0) {
    states.push_back("KL");
    if(this->lane > 0){states.push_back("LCL");}
    if(this->lane < 2){states.push_back("LCR");}
  } else if (state.compare("LCL") == 0) {
    states.push_back("KL");
  } else if (state.compare("LCR") == 0) {
    states.push_back("KL");
  }
    
  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}



