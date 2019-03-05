#include "cost.h"
#include <cmath>
#include <functional>
//#include <iterator>
//#include <map>
#include <string>
#include <vector>
//#include "vehicle.h"
#include "helpers.h"

using std::string;
using std::vector;

MAX_SPEED = 49.5;

double collision_cost(vector<vector<double>> &sensor_fusion, vector<vector<double>> &trajectory, 
                      vector<double> &map_waypoints_s, vector<double> &map_waypoints_x, 
                      vector<double> &map_waypoints_y)
{
  vector<double> traj_x = trajectory[0];
  vector<double> traj_y = trajectory[1];
  

  for(int i = 0; i < traj_x.size(); i++)
  {
    double t = .02*(i+1);
    double min_dist = 999999999.0;
    for(j = 0; j < sensor_fusion.size(); j++)
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
      double dist = distance(traj_x, traj_y, x, y);
      if(dist<min_dist)
      {
        min_dist = dist;
      }

    }
    if(min_dist < vehicle.safe_radius)
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
  double ave_vel = distance(traj_x[-1], traj_y[-1], traj_x[0], traj_y[0]);
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
  
  double collision_cost = collision_cost(sensor_fusion, trajectory, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  double ave_vel_cost = ave_vel_cost(trajectory);

  double cost;

  cost = 10 * collision_cost + ave_vel_cost;

  return cost;
}

