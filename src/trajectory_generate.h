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


/*
vector<vector<double>> traj_gen(Vehicle &my_car, string &state, double &ref_vel,
                                        vector<double> &previous_path_x, vector<double> &previous_path_y, 
                                        vector<double> &map_waypoints_s, vector<double> &map_waypoints_x, 
                                        vector<double> &map_waypoints_y)
{
  vector<vector<double>> prev_traj_sd;
  vector<vector<double>> prev_traj_xy;

  prev_traj_xy.push_back(previous_path_x);
  prev_traj_xy.push_back(previous_path_y);

  prev_traj_sd = xy_to_sd(my_car, prev_traj_xy, map_waypoints_x, map_waypoints_y);

  vector<double> prev_traj_s = prev_traj_sd[0];
  vector<double> prev_traj_d = prev_traj_sd[1];

  int prev_size = prev_traj_s.size();

  vector<double> ptsx;
  vector<double> ptsy;

  double prev_car_x;
  double prev_car_y;

  double car_s = my_car.s;
  double car_d = my_car.d;

  double car_x = my_car.x;
  double car_y = my_car.y;

  double end_path_s;

  if(prev_size < 2)
  {
    prev_car_x = my_car.x - cos(my_car.yaw);
    prev_car_y = my_car.y - sin(my_car.yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);

    end_path_s = car_s;
  }
  else
  {
    car_x = previous_path_x[prev_size-1];
    prev_car_x = previous_path_x[prev_size-2];

    car_y = previous_path_y[prev_size-1];
    prev_car_y = previous_path_y[prev_size-2];

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);

    end_path_s = prev_traj_s[prev_size-1];
  }

  int lane = int(floor(car_d/4));
  int target_lane;

  if(state.compare("KL") == 0)
  {
    target_lane = lane;
  }
  else if(state.compare("LCL") == 0)
  {
    target_lane = lane - 1;
  }
  else if(state.compare("LCR") == 0)
  {
    target_lane = lane + 1;
  }

  

  vector<double> next_wp0 = getXY(end_path_s+ref_vel/2.24, target_lane*4+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(end_path_s+2*ref_vel/2.24, target_lane*4+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(end_path_s+3*ref_vel/2.24, target_lane*4+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);


  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  tk::spline s;
  s.set_points(ptsx, ptsy);

  vector<double> traj_x;
  vector<double> traj_y;

  //int idx;
  //if(prev_size < 10){idx = prev_size;}
  //else{idx = 10;}



  for(int i = 0; i < prev_size; i++)
  {
    traj_x.push_back(previous_path_x[i]);
    traj_y.push_back(previous_path_y[i]);
  }
  
  
  

  double target_x = previous_path_x[prev_size-1] + 30.0;
  double target_y = s(target_x);
  double target_dist = distance(previous_path_x[prev_size-1], previous_path_y[prev_size-1], target_x, target_y);

  //double x_add_on = 0;

  double N = (target_dist/(0.02*ref_vel/2.24));

  for(int i = 1; i <= 50-prev_size; i++)
  {
              
    double x_point = car_x + i*(target_x)/N;
    double y_point = s(x_point);

    

    traj_x.push_back(x_point);
    traj_y.push_back(y_point);

  }

  vector<vector<double>> traj;
  traj.push_back(traj_x);
  traj.push_back(traj_y);

  //for(int j = 0; j < 3; j++)  // For each state, generate traj with 3 different velocity
  //{
    
  //  trajectroies.push_back(traj);
  //}

  
  
  return traj;



}
*/






vector<vector<double>> traj_gen(double &end_path_d, Vehicle &my_car, string &state, double &ref_vel,
                                        vector<double> &previous_path_x, vector<double> &previous_path_y, 
                                        vector<double> &map_waypoints_s, vector<double> &map_waypoints_x, 
                                        vector<double> &map_waypoints_y)
{
  


  //std::cout<<"DEBUG!";

  //vector<vector<vector<double>>> trajectroies;
  double car_x = my_car.x;
  double car_y = my_car.y;
  double car_yaw = my_car.yaw;
  double car_s = my_car.s;

  vector<double> ptsx;
  vector<double> ptsy;

  int prev_size = previous_path_x.size(); 
  double ref_yaw = deg2rad(car_yaw);
  double ref_x = car_x;
  double ref_y = car_y;

    //if(v == 0 && i == 0){break;}

  int lane = int(floor(end_path_d/4));
  if(lane < 0){lane = 0;}
  else if(lane > 2){lane = 2;}
  int target_lane;



  //double prev_car_x = car_x - cos(car_yaw);
  //double prev_car_y = car_y - sin(car_yaw);

  //ptsx.push_back(prev_car_x);
  //ptsx.push_back(car_x);

  //ptsy.push_back(prev_car_y);
  //ptsy.push_back(car_y);
  
  //std::cout<<"DEBUG!";
  
  if(prev_size < 2)
  {
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

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
  
  
  double s_plus;
  

  if(state.compare("KL") == 0 || state.compare("PLCL") == 0 || state.compare("PLCR") == 0)
  {
    target_lane = lane;
    s_plus = 30;
  }
  else if(state.compare("LCL") == 0)
  {
    target_lane = lane - 1;
    s_plus = 10;
  }
  else if(state.compare("LCR") == 0)
  {
    target_lane = lane + 1;
    s_plus = 10;
  }

  //double ref_len = ref_vel*10;

  vector<double> next_wp0 = getXY(car_s+s_plus, (2+4*target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s+2*s_plus, (2+4*target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s+3*s_plus, (2+4*target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

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

  //int idx;
  //if(prev_size < 10){idx = prev_size;}
  //else{idx = 10;}



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

  //for(int j = 0; j < 3; j++)  // For each state, generate traj with 3 different velocity
  //{
    
  //  trajectroies.push_back(traj);
  //}

  
  
  return traj;

}


