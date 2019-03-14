#ifndef COST_H
#define COST_H

//#include "vehicle.h"
#include <vector>
#include "helpers.h"
#include "vehicle.h"

using std::map;
using std::string;
using std::vector;


#define MAX_SPEED 49.5
#define SAFE_RADIUS 5.0
#define COLLISION_RADIUS 10.0
#define FOLLOW_DIST 30.0
#define MAX_ACC 10.0
#define MAX_JERK 5.0

vector<vector<double>> xy_to_sd(Vehicle &my_car, vector<vector<double>> &trajectory, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y)
{
  vector<double> traj_x = trajectory[0];
  vector<double> traj_y = trajectory[1];
  vector<double> sd;
  double theta;
  double prev_x, prev_y;
  double x, y;
  vector<double> traj_s, traj_d;
  vector<vector<double>> traj_sd;

  for(int i = 0; i < traj_x.size(); i++)
  {
    x = traj_x[i];
    y = traj_y[i];
    if(i == 0){theta = deg2rad(my_car.yaw);}
    else{theta = atan2(y-prev_y, x-prev_x);}
    sd = getFrenet(traj_x[i], traj_y[i], theta, map_waypoints_x, map_waypoints_y);
    traj_s.push_back(sd[0]);
    traj_d.push_back(sd[1]);
    prev_x = x;
    prev_y = y;
  }

  traj_sd.push_back(traj_s);
  traj_sd.push_back(traj_d);

  return traj_sd;
}

double collision_cost(Vehicle &my_car, vector<Vehicle> &other_cars, vector<vector<double>> &trajectory)
{
  vector<double> traj_x = trajectory[0];
  vector<double> traj_y = trajectory[1];
  //vector<double> traj_s = traj_sd[0];
  //vector<double> traj_d = traj_sd[1];

  for(int i = 0; i < traj_x.size(); i++)
  {
    double t = 0.02*(i+1);
    for(int j = 0; j < other_cars.size(); j++)
    {
      double dist = distance(other_cars[j].x, other_cars[j].y, traj_x[i], traj_y[i]);
      if(dist < COLLISION_RADIUS){return 1;}
      
    }
      
    
  }
  return 0;
}


double too_close_cost(Vehicle &my_car, vector<Vehicle> &other_cars, vector<vector<double>> &trajectory)
{
  double cost;
  double min_dist = 999999999999.0;
  vector<double> traj_x = trajectory[0];
  vector<double> traj_y = trajectory[1];


  for(int i = 0; i < traj_x.size(); i++)
  {
    double t = 0.02*(i+1);
    for(int j = 0; j < other_cars.size(); j++)
    {
      double other_car_x = other_cars[j].x + other_cars[j].vx * t;
      double other_car_y = other_cars[j].y + other_cars[j].vy * t;

      
      double dist = distance(other_car_x, other_car_y, traj_x[i], traj_y[i]);

      if(dist < min_dist)
      {
        min_dist = dist;
      }
    }
  }
  if(min_dist > 120)
  {
    cost = 0;
  }
  else if(min_dist < SAFE_RADIUS)
  {
    cost = 1;
  }
  else
  {
    double x = 2*SAFE_RADIUS/min_dist;
    cost = 2.0/(1+exp(-x))-1.0;
  }
  return cost;
}

double follow_cost(Vehicle &my_car, vector<Vehicle> &other_cars, vector<vector<double>> &traj_sd)
{
  vector<double> traj_s = traj_sd[0];
  vector<double> traj_d = traj_sd[1];
  for(int i = 0; i < other_cars.size(); i++)
  {
    if(other_cars[i].lane == int(floor(traj_d[traj_d.size()-1]/4)))
    {
      if(other_cars[i].s - traj_s[traj_s.size()-1]>0)
      {
        if(other_cars[i].s - traj_s[traj_s.size()-1]<=FOLLOW_DIST){return 1;}
      }
      
    }
  }
  return 0;

}


double ave_vel_cost(vector<vector<double>> &trajectory)
{
  vector<double> traj_x = trajectory[0];
  vector<double> traj_y = trajectory[1];

  double t = traj_x.size()*0.02;
  double total_dist = 0.0;
  for(int i = 0; i < (traj_x.size()-1); i++)
  {
  	total_dist += distance(traj_x[i], traj_y[i], traj_x[i+1], traj_y[i+1]);
  }
  double ave_vel = total_dist/t;
  double cost;
  if(ave_vel <= MAX_SPEED)
  {
    cost = (MAX_SPEED - ave_vel)/MAX_SPEED;
  }
  else
  {
    cost = 1;
  }
  return cost;
}

double max_vel_cost(vector<vector<double>> &trajectory)
{
  vector<double> traj_x = trajectory[0];
  vector<double> traj_y = trajectory[1];
  double max_vel = 0;
  for(int i = 0; i < (traj_x.size()-1); i ++)
  {
    double instance_vel = distance(traj_x[i], traj_y[i], traj_x[i+1], traj_y[i+1])/0.02;
    if(instance_vel > MAX_SPEED)
    {
      return 1;
    }
  }
  return 0;
 
}


vector<double> traj_deriv(vector<double> &traj_x)
{
  vector<double> traj_vx;


  for(int i = 0; i < traj_x.size()-1; i++)
  {
    double dist = fabs(traj_x[i] - traj_x[i+1]);
    double vx = dist/0.02;
    traj_vx.push_back(vx);
  }
  return traj_vx;
}
/*
vector<double> traj_deriv_2(vector<double> &traj_vel)
{
  vector<double> traj_deriv_2;
  for(int i = 0; i < traj_vel.size()-1; i++)
  {
    double acc = (traj_vel[i+1] - traj_vel[i])/0.02;
    traj_deriv_2.push_back(acc);
  }
  return traj_deriv_2;
}
*/

double max_acc_cost(vector<vector<double>> &trajectory)
{
  vector<double> traj_x = trajectory[0];
  vector<double> traj_y = trajectory[1];

  vector<double> traj_vx = traj_deriv(traj_x);
  vector<double> traj_vy = traj_deriv(traj_y);

  vector<double> traj_ax = traj_deriv(traj_vx);
  vector<double> traj_ay = traj_deriv(traj_vy);

  for(int i = 0; i < traj_ax.size(); i++)
  {
    double acc = sqrt(traj_ax[i]*traj_ax[i]+traj_ay[i]*traj_ay[i]);
    if(acc > MAX_ACC){return 1;}
  }
  return 0;

}

double max_jerk_cost(vector<vector<double>> &trajectory)
{
  vector<double> traj_x = trajectory[0];
  vector<double> traj_y = trajectory[1];

  vector<double> traj_vx = traj_deriv(traj_x);
  vector<double> traj_vy = traj_deriv(traj_y);

  vector<double> traj_ax = traj_deriv(traj_vx);
  vector<double> traj_ay = traj_deriv(traj_vy);

  vector<double> traj_jx = traj_deriv(traj_ax);
  vector<double> traj_jy = traj_deriv(traj_ay);

  for(int i = 0; i < traj_jx.size();i++)
  {
    double jerk = sqrt(traj_jx[i]*traj_jx[i]+traj_jy[i]*traj_jy[i]);
    if(jerk > MAX_JERK){return 1;}
  }
  return 0;
}

double stay_in_lane_cost(Vehicle &my_car, vector<vector<double>> &traj_sd)
{
  vector<double> traj_s = traj_sd[0];
  vector<double> traj_d = traj_sd[1];
  //double total_diff = 0.0;

  for(int i = 0; i <traj_d.size(); i++)
  {
    
    if(traj_d[i] != my_car.lane*4+2){return 1;}
  }
  
  return 0;
}

/*
double smooth_traj_cost(vector<vector<double>> &traj_sd)
{
  vector<double> traj_s = traj_sd[0];
  vector<double> traj_d = traj_sd[1];
  double d_diff, prev_d_diff;

  for(int i = 0; i < traj_d.size()-1; i++)
  {
    d_diff = traj_d[i] - traj_d[i+1];
    if(i != 0)
    {
      if(d_diff*prev_d_diff<0)
      {
        return 1;
      }
    }
    prev_d_diff = d_diff;
  }
  return 0;
}
*/


double calculate_cost(Vehicle &my_car, vector<Vehicle> &other_cars, vector<vector<double>> &trajectory, 
                      vector<double> &map_waypoints_x, vector<double> &map_waypoints_y) 
{
  // Sum weighted cost functions to get total cost for trajectory.
  vector<vector<double>> traj_sd = xy_to_sd(my_car, trajectory, map_waypoints_x, map_waypoints_y);


  double col = collision_cost(my_car, other_cars, trajectory);
  double tc = too_close_cost(my_car, other_cars, trajectory);
  double fo = follow_cost(my_car, other_cars, traj_sd);
  double avv = ave_vel_cost(trajectory);
  double mv = max_vel_cost(trajectory);
  double macc = max_acc_cost(trajectory);
  double mjerk = max_jerk_cost(trajectory);
  double slc = stay_in_lane_cost(my_car, traj_sd);
  //double stc = smooth_traj_cost(traj_sd);

  double cost;

  cost = 9999 * col 
         + 10*tc 
         + 100*fo 
         + 1000 * avv 
         + 9999 * mv 
         + 100 * macc 
         + 100 * mjerk 
         + 1000 * slc;
  //cost = col + tc + fo + avv + mv + macc + mjerk;

  return cost;
}


#endif //COST_H