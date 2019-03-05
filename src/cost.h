#ifndef COST_H
#define COST_H

//#include "vehicle.h"
#include <vector>
#include "helpers.h"

using std::map;
using std::string;
using std::vector;


double MAX_SPEED = 49.5;
double SAFE_RADIUS = 10.0;

double collision_cost(vector<vector<double>> &sensor_fusion, vector<vector<double>> &trajectory, 
                      vector<double> &map_waypoints_s, vector<double> &map_waypoints_x, 
                      vector<double> &map_waypoints_y)
{
  vector<double> traj_x = trajectory[0];
  vector<double> traj_y = trajectory[1];
  

  for(int i = 0; i < traj_x.size(); i++)
  {
    double t = 0.02*(i+1);
    double min_dist = 999999.0;
    for(int j = 0; j < sensor_fusion.size(); j++)
    {
      double x, y, s, d;
      vector<double> xy;
      s = sensor_fusion[j][5];
      d = sensor_fusion[j][6];
      xy = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      x = xy[0];
      y = xy[1];
      double vx = sensor_fusion[j][3];
      double vy = sensor_fusion[j][4];
      x += vx*t;
      y += vy*t;
      double dist = distance(traj_x[i], traj_y[i], x, y);
      if(dist<min_dist)
      {
        min_dist = dist;
      }

    }
    if(min_dist < SAFE_RADIUS)
    {
      return 1;
    }
  }
  return 0;
}


double ave_vel_cost(vector<vector<double>> &trajectory)
{
  vector<double> traj_x = trajectory[0];
  vector<double> traj_y = trajectory[1];

  double t = traj_x.size()*.02;
  double total_dist = 0.0;
  for(int i = 0; i < (traj_x.size()-1); i++)
  {
  	total_dist += distance(traj_x[i], traj_y[i], traj_x[i+1], traj_y[i+1]);
  }
  double ave_vel = total_dist/t;
  double cost;
  if(ave_vel <= MAX_SPEED)
  {
    cost = 1 - exp(MAX_SPEED - ave_vel);
  }
  else
  {
    cost = 1;
  }
  return cost;
}






double calculate_cost(vector<vector<double>> &sensor_fusion, vector<vector<double>> &trajectory, 
                      vector<double> &map_waypoints_s, vector<double> &map_waypoints_x, 
                      vector<double> &map_waypoints_y) 
{
  // Sum weighted cost functions to get total cost for trajectory.
  
  double col = collision_cost(sensor_fusion, trajectory, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  double ave_vel = ave_vel_cost(trajectory);

  double cost;

  cost = 10.0 * col + ave_vel;

  return cost;
}


#endif //COST_H