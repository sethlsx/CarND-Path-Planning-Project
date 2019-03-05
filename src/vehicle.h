#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;



class Vehicle
{
public:
	int lane;
	double x;
	double y;
	double d;
	double s;
	double v; 
	double vx;
	double vy;
	double yaw;
	string state;
	int safe_radius;
	//Constructors
	Vehicle();
	Vehicle(int lane, double x, double y, double d, double s, double v, double vx, double vy, double yaw); // KL stands for keep lane

	//Destructor
	virtual ~Vehicle();

	//Vehicle functions
	
	vector<string> successor_states();

	
};

#endif  //VEHICLE_H