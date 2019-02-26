#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;


string fsm(int &lane, vector<vector<double>> &sensor_fusion, double &car_s, double &car_d, int &t)
{
	string status;
	bool left_clear = true;
	bool right_clear = true;
	bool too_close = false;
	for(int i = 0; i < sensor_fusion.size(); i++)
	{
		float d = sensor_fusion[i][6];

		//check the lane car is currently in
        if(d < (2+4*lane+2) && d > (2+4*lane-2))
        {
          double vx = sensor_fusion[i][3];
          double vy = sensor_fusion[i][4];
          double check_speed = sqrt(vx*vx+vy*vy);
          double check_car_s = sensor_fusion[i][5];

          check_car_s+=((double)t*.02*check_speed);

          if((check_car_s > car_s) && ((check_car_s-car_s) < 30))
          {
            too_close = true;
          }
        }
    
    
		//check the left lane
		if(d <= (2+4*lane-2) && d > 0) 
		{
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
        	double check_speed = sqrt(vx*vx+vy*vy);
        	double check_car_s = sensor_fusion[i][5];

        	check_car_s+=((double)t*.02*check_speed);
        	if((check_car_s - car_s < 30) || (car_s - check_car_s < 10))
        	{
            	left_clear = false;
        	}
		}
		else if(d >= (2+4*lane+2))
		{
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
        	double check_speed = sqrt(vx*vx+vy*vy);
        	double check_car_s = sensor_fusion[i][5];

        	check_car_s+=((double)t*.02*check_speed);
        	if((check_car_s - car_s < 30) || (car_s - check_car_s < 10))
        	{
            	right_clear = false;
        	}

		}
	}

	if(!too_close) //if not close up to any viehcle, just keep straight
    {
        status = "keep true";
        //std::cout << status;
        return status;
    }

	
	if(left_clear && (lane != 0)) //if the left lane is clear, change to the left lane;
	{
		status = "CL";

	}
	else if(right_clear && (lane != 2))//if the right lane is clear, change to the right lane;
	{
		status = "CR";
	} 
	else
	{
		status = "PLCL/R";
	}
	//std::cout << status;
	return status;	
}