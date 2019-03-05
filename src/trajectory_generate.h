
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
#include "vehicle.h"
#include "cost.h"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

vector<vector<vector<double>>> traj_gen(double &car_x, double &car_y, double &car_yaw, double &car_s, 
                                        string &state, int &lane, double &v,
                                        vector<double> &previous_path_x, vector<double> &previous_path_y, 
                                        vector<double> &map_waypoints_s, vector<double> &map_waypoints_x, 
                                        vector<double> &map_waypoints_y)
{
  vector<vector<vector<double>>> trajectroies;

  for(int i = 0; i < 3; i++)  // For each state, generate traj with 3 different velocity
  {
    vector<double> ptsx;
    vector<double> ptsy;

    int prev_size = previous_path_x.size(); 
    double ref_yaw = deg2rad(car_yaw);
    double ref_x = car_x;
    double ref_y = car_y;

    if(v == 0 && i == 0){break;}

    double ref_vel = v + (i-1) * .224;

    if(ref_vel == 0)
    {
      ref_vel += .224;
    }

    if(prev_size < 2)
    {
      double prev_car_x = car_x - cos(ref_yaw);
      double prev_car_y = car_y - sin(ref_yaw);

      ptsx.push_back(prev_car_x);
      ptsx.push_back(car_x);

      ptsy.push_back(prev_car_y);
      ptsy.push_back(car_y);
    }
    else
    {
      ref_x = previous_path_x[prev_size-1];
      ref_y = previous_path_y[prev_size-1];

      double ref_x_prev = previous_path_x[prev_size-2];
      double ref_y_prev = previous_path_y[prev_size-2];
      ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

      ptsx.push_back(ref_x_prev);
      ptsx.push_back(ref_x);

      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);

    }

    vector<vector<double>> next_wps;

    if(state.compare("KL") == 0)
    {
      lane = lane;
    }
    else if(state.compare("LCL") == 0)
    {
      lane = lane - 1;
    }
    else if(state.compare("LCR") == 0)
    {
      lane = lane + 1;
    }

    vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for(int i = 0; i < ptsx.size(); i++)
    {
      double shift_x = ptsx[i] - ref_x;
      double shift_y = ptsy[i] - ref_y;

      ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
      ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));

    }

            

    tk::spline s;

    s.set_points(ptsx, ptsy);

    vector<double> traj_x;
    vector<double> traj_y;

    for(int i = 0; i < prev_size; i++)
    {
      traj_x.push_back(previous_path_x[i]);
      traj_y.push_back(previous_path_y[i]);
    }

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

    double x_add_on = 0;

    double N = (target_dist/(.02*ref_vel/2.24));

    for(int i = 1; i <= 50-prev_size; i++)
    {
              
      double x_point = x_add_on + (target_x)/N;
      double y_point = s(x_point);

      x_add_on = x_point;

      double x_ref = x_point;
      double y_ref = y_point;

      x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
      y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));

      x_point += ref_x;
      y_point += ref_y;

      traj_x.push_back(x_point);
      traj_y.push_back(y_point);

    }
    vector<vector<double>> traj;
    traj.push_back(traj_x);
    traj.push_back(traj_y);
    trajectroies.push_back(traj);
  }

  
  
  return trajectroies;

}


