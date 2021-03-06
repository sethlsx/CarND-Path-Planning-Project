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
#define SAFE_RADIUS 2.0
#define COLLISION_RADIUS 1.0
#define FOLLOW_DIST 50.0
#define MAX_ACC 10.0
#define MAX_JERK 10.0

double get_lane_vel(vector<Vehicle> &other_cars, int &lane)
{
  double lane_vel = -1.0;
  for(int i = 0; i < other_cars.size(); i++)
  {
    if(other_cars[i].lane == lane)
    {
      if(other_cars[i].v > lane_vel){lane_vel = other_cars[i].v;}
    }
  }


  return lane_vel;
}

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
      double dist = distance(other_cars[j].x + other_cars[j].vx*t, other_cars[j].y+other_cars[j].vy*t, traj_x[i], traj_y[i]);
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
  if(min_dist > 2*SAFE_RADIUS)
  {
    cost = 0;
  }
  else if(min_dist <= SAFE_RADIUS)
  {
    cost = 1;
  }
  else
  {
    cost = 2 * SAFE_RADIUS/min_dist - 1;
  }
  return cost;
}

double follow_cost(Vehicle &my_car, vector<Vehicle> &other_cars, vector<vector<double>> &traj_sd)
{
  vector<double> traj_s = traj_sd[0];
  vector<double> traj_d = traj_sd[1];

  double min_dist = 120.0;
  double t = traj_sd.size()*0.02;
  for(int i = 0; i < other_cars.size(); i++)
  {
    if(other_cars[i].lane == int(floor(traj_d[traj_d.size()-1]/4)))
    {
      if(other_cars[i].s - my_car.s>0)
      {
        if(other_cars[i].s + other_cars[i].v*t- traj_s[traj_s.size()-1] > 0)
        {
          if(other_cars[i].s + other_cars[i].v*t- traj_s[traj_s.size()-1] < min_dist)
          {
            min_dist = other_cars[i].s - traj_s[traj_s.size()-1];
          }
        }
        else
        {
          return 1; 
        }
        
      }
      
    }
  }

  double cost;

  if(min_dist >= 120)
  {
    cost = 0;
  }
  else if(min_dist < SAFE_RADIUS)
  {
    cost = 1;
  }
  else
  {
    double x = 120.0/min_dist-1;
    cost = 2/(1+exp(-x))-1;
  }

  return cost;

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
    double dist = traj_x[i+1] - traj_x[i];
    double vx = dist/0.2;
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

double max_acc_cost(vector<vector<double>> &traj_sd)
{
  vector<double> traj_s = traj_sd[0];
  vector<double> traj_d = traj_sd[1];

  vector<double> traj_vs = traj_deriv(traj_s);
  vector<double> traj_vd = traj_deriv(traj_d);

  vector<double> traj_as = traj_deriv(traj_vs);
  vector<double> traj_ad = traj_deriv(traj_vd);

  for(int i = 0; i < traj_as.size(); i++)
  {
    if(fabs(traj_as[i]) > MAX_ACC/2.24){return 1;}
    if(fabs(traj_ad[i]) > MAX_ACC/2.24){return 1;}
  }
  return 0;

}

double max_jerk_cost(vector<vector<double>> &traj_sd)
{
  vector<double> traj_s = traj_sd[0];
  vector<double> traj_d = traj_sd[1];

  vector<double> traj_vs = traj_deriv(traj_s);
  vector<double> traj_vd = traj_deriv(traj_d);

  vector<double> traj_as = traj_deriv(traj_vs);
  vector<double> traj_ad = traj_deriv(traj_vd);

  vector<double> traj_js = traj_deriv(traj_as);
  vector<double> traj_jd = traj_deriv(traj_ad);

  for(int i = 0; i < traj_js.size();i++)
  {
    if(fabs(traj_js[i]) > MAX_JERK/2.24){return 1;}
    if(fabs(traj_jd[i]) > MAX_JERK/2.24){return 1;}
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
    
    if(fabs(traj_d[i] - my_car.lane*4-2)>1){return 1;}
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

double front_collison_time_cost(Vehicle &my_car, vector<Vehicle> &other_cars, vector<vector<double>> trajectory)
{
  double cost = 0;
  int col_idx;
  if(collision_cost(my_car, other_cars, trajectory))
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
        double dist = distance(other_cars[j].x + other_cars[j].vx*t, other_cars[j].y+other_cars[j].vy*t, traj_x[i], traj_y[i]);
        if(dist < COLLISION_RADIUS){col_idx = j;}
      
      }
      
    
    }
    if(other_cars[col_idx].s > my_car.s)
    {
      cost = 1-ave_vel_cost(trajectory);  
    }
    
  }
  return cost;
}

double efficiency_cost(vector<vector<double>> &traj_sd)
{
  vector<double> traj_s = traj_sd[0];
  vector<double> traj_d = traj_sd[1];
  double t = traj_s.size()*0.02;
  double vs = (traj_s[traj_s.size()-1] - traj_s[0])/(t*2.24);

  double cost;

  if(vs <= MAX_SPEED)
  {
    cost = (MAX_SPEED - vs)/MAX_SPEED;
  }
  else
  {
    cost = 1;
  }
  return cost;
}


double calculate_cost(Vehicle &my_car, vector<Vehicle> &other_cars, vector<vector<double>> &trajectory, 
                      vector<double> &map_waypoints_x, vector<double> &map_waypoints_y) 
{
  // Sum weighted cost functions to get total cost for trajectory.
  vector<vector<double>> traj_sd = xy_to_sd(my_car, trajectory, map_waypoints_x, map_waypoints_y);


  double col = collision_cost(my_car, other_cars, trajectory);
  //double tc = too_close_cost(my_car, other_cars, trajectory);
  //double fo = follow_cost(my_car, other_cars, traj_sd);
  double avv = ave_vel_cost(trajectory);
  //double mv = max_vel_cost(trajectory);
  double macc = max_acc_cost(traj_sd);
  double mjerk = max_jerk_cost(traj_sd);
  double slc = stay_in_lane_cost(my_car, traj_sd);
  //double stc = smooth_traj_cost(traj_sd);
  double ctc = front_collison_time_cost(my_car, other_cars, trajectory);
  double ec = efficiency_cost(traj_sd);

  double cost;

  cost = 99999 * col 
         //+ 10 * tc 
         //+ 1000 * fo 
         //+ 500 * avv 
         //+ 99999 * mv 
         + 99999 * macc 
         + 99999 * mjerk 
         + 100 * slc
         + 10000 * ctc
         + 500 * ec
         ;
  //cost = col + tc + fo + avv + mv + macc + mjerk;

  return cost;
}


#endif //COST_H