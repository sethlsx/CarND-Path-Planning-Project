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
#include <math.h>
//#include <limits.h>
#include "cost.h"
#include "vehicle.h"
// #include "fsm.h" //Finite State Machines for Behavior Planning
//#include "trajectory_generate.h" //Hide the trajectory generate code in a head file

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


int main() {
  uWS::Hub h;
  //std::cout << "DEBUG";
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  
  //string status = "keep true";
  string state = "KL";

  h.onMessage([&state, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
                
    

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          //create our vehicle
          Vehicle my_car = Vehicle();

          my_car.lane = int(floor(car_d/4));

          my_car.x = car_x;
          my_car.y = car_y;
          my_car.s = car_s;
          my_car.d = car_d;
          my_car.yaw = car_yaw;
          my_car.v = car_speed;
          
          //std::cout<<my_car.yaw;
          
          //int lane = int(floor(car_d/4));
          //double x = car_x;
          //double y = car_y;
          //double d = car_d;
          //double s = car_s;
          //double v = car_speed;
          //double yaw = car_yaw;
          //Vehicle ego_car = Vehicle(lane, x, y, d, s, v, 0, 0, yaw);
          
          vector<double> alt_previous_path_x, alt_previous_path_y;

          for(int i = 0; i < previous_path_x.size(); i++)
          {
            alt_previous_path_x.push_back(previous_path_x[i]);
            alt_previous_path_y.push_back(previous_path_y[i]);
          }




          //  transfer the data in sensor_fusion into a vector of vehicles

          //vector<vector<double>> alt_sensor_fusion;

          vector<Vehicle> other_vehicles;
          Vehicle one_vehicle = Vehicle();
          

          for(int i = 0; i < sensor_fusion.size(); i++)
          {
             one_vehicle.d = sensor_fusion[i][6];
             one_vehicle.lane = int(floor(one_vehicle.d/4));
             one_vehicle.s = sensor_fusion[i][5];
             vector<double> xy;
             xy = getXY(one_vehicle.s, one_vehicle.d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
             one_vehicle.x = xy[0];
             one_vehicle.y = xy[1];
             double vx = sensor_fusion[i][3];
             double vy = sensor_fusion[i][4];
             one_vehicle.v = sqrt(vx*vx+vy*vy);

             //Vehicle other_vehicle = Vehicle(lane, x, y, d, s, v, vx, vy, yaw);
             other_vehicles.push_back(one_vehicle);
             //alt_sensor_fusion.push_back(sensor_fusion[i]);
          }

          //std::cout << "DEBUG";

          vector<string> available_states;
          double min_cost = 999999999999999.0;
          vector<vector<double>> final_traj;
          //string state = my_car.state;
          int lane = my_car.lane;
          //double lane_speed = -1.0;
          int temp_lane;
          int end_lane = int(floor(end_path_d/4));
          bool left_lane_empty = false;
          bool right_lane_empty = false;

          if(end_lane > 0)
          {
            temp_lane = end_lane - 1;
            left_lane_empty = target_lane_empty(temp_lane, my_car, other_vehicles);
          }
          if(end_lane < 2)
          {
            temp_lane = end_lane + 1;
            right_lane_empty = target_lane_empty(temp_lane, my_car, other_vehicles);
          }

          if(state.compare("KL") == 0)
          {
            available_states.push_back("KL");
            if(left_lane_empty){available_states.push_back("LCL");}//else{std::cout<<"Car on the left!\n";}
            if(right_lane_empty){available_states.push_back("LCR");}//else{std::cout<<"Car on the right!\n";}
          }
          else if(state.compare("LCL") == 0)
          {
            available_states.push_back("KL");
            if(left_lane_empty){available_states.push_back("LCL");}//else{std::cout<<"Car on the left!\n";}
          }
          else if(state.compare("LCR") == 0)
          {
            available_states.push_back("KL");
            if(right_lane_empty){available_states.push_back("LCR");}//else{std::cout<<"Car on the right!\n";}
          }


          //std::cout << "DEBUG";
          /*
          if(state.compare("KL") == 0) 
          {
            available_states.push_back("KL");
            if(end_lane > 0){available_states.push_back("PLCL");}
            if(end_lane < 2){available_states.push_back("PLCR");}
          } else if(state.compare("PLCL") == 0){
            available_states.push_back("KL");
            available_states.push_back("PLCL");
            //temp_lane = lane - 1;
            //lane_speed = get_lane_vel(other_vehicles, temp_lane);
            available_states.push_back("LCL");
          } else if(state.compare("PLCR") == 0){
            available_states.push_back("KL");
            available_states.push_back("PLCR");
            //temp_lane = lane + 1;
            //lane_speed = get_lane_vel(other_vehicles, temp_lane);
            available_states.push_back("LCR");
          } else if (state.compare("LCL") == 0) {
            available_states.push_back("KL");
            if(end_lane > 0){available_states.push_back("LCL");}
          } else if (state.compare("LCR") == 0) {
            available_states.push_back("KL");
            if(end_lane < 2){available_states.push_back("LCR");}
          }
          */



          //available_states = my_car.successor_states();
          string final_state;
          string temp_state;

          double target_speed;
          int t_lane;

          //std::cout << "There are " << available_states.size() << " available states, they are:\n";
/*
          for(int i = 0; i < available_states.size(); i++)
          {
            std::cout << available_states[i] << "\n";

          }
*/
          for(int i = 0; i < available_states.size(); i++)
          {
            //std::cout << "Current state: " << available_states[i] << "\n";

            double ref_speed = MAX_SPEED;
            temp_state = available_states[i];
              //std::cout << temp_state << ":\n";
            if(temp_state.compare("KL") == 0)
            {
              t_lane = my_car.lane;
            }
            else if(temp_state.compare("LCL") == 0)
            {
              t_lane = my_car.lane - 1;
            }
            else if(temp_state.compare("LCR") == 0)
            {
              t_lane = my_car.lane + 1;
            }

            //std::cout << "CHECK POINT -1\n";
            
            Vehicle lead_car = get_lane_leading_car(t_lane, my_car, other_vehicles);

            //std::cout << "CHECK POINT 0\n";

            if(lead_car.v != -1 && lead_car.s - my_car.s < 60)
            {
              ref_speed = lead_car.v;
            }
            else
            {
              ref_speed = MAX_SPEED;
            }

           //std::cout << "CHECK POINT 0.5\n";
            /*
            
            double min_s = 9999999;
            double min_s1 = 9999999;
            double nearest_v = 0;
            double v1 = -1.0;
            double v2 = -1.0;
            for(int j = 0; j < other_vehicles.size(); j++)
            {
              if(temp_state.compare("PLCL") == 0)
              {
                if(other_vehicles[j].lane == t_lane)
                {
                  if(fabs(other_vehicles[j].s - my_car.s) < min_s)
                  {
                    min_s = fabs(other_vehicles[j].s - my_car.s);
                    if(min_s < 60){v1 = other_vehicles[j].v;}
                    
                  }
                }
                else if(other_vehicles[j].lane == t_lane + 1)
                {
                  if(other_vehicles[j].s - my_car.s > 0)
                  {
                    if(other_vehicles[j].s - my_car.s < min_s1)
                    {
                      min_s1 = other_vehicles[j].s - my_car.s;
                      if(min_s1 < 60){v2 = other_vehicles[j].v;}
                    }
                  }
                }
              }
              else if(temp_state.compare("PLCR") == 0)
              {
                if(other_vehicles[j].lane == t_lane)
                {
                  if(fabs(other_vehicles[j].s - my_car.s) < min_s)
                  {
                    min_s = fabs(other_vehicles[j].s - my_car.s);
                    if(min_s < 60){v1 = other_vehicles[j].v;}
                  }
                }
                else if(other_vehicles[j].lane == t_lane - 1)
                {
                  if(other_vehicles[j].s - my_car.s > 0)
                  {
                    if(other_vehicles[j].s - my_car.s < min_s1)
                    {
                      min_s1 = other_vehicles[j].s - my_car.s;
                      if(min_s1 < 60){v2 = other_vehicles[j].v;}
                    }
                  }
                }
              }
              else
              {
                if(other_vehicles[j].lane == t_lane)
                {
                  if((temp_state.compare("KL") != 0))
                  {
                    if(fabs(other_vehicles[j].s - my_car.s) < min_s)
                    {
                      min_s = fabs(other_vehicles[j].s - my_car.s);
                      ref_speed = other_vehicles[j].v;
                    }
                  }
                  else
                  {
                    if(other_vehicles[j].s - my_car.s > 0)
                    {
                      if(other_vehicles[j].s - my_car.s < min_s)
                      {
                        min_s = other_vehicles[j].s - my_car.s;
                        nearest_v = other_vehicles[j].v;
                      }
                    }
                  }
                }
              }
              
            }
            if(min_s < 60 && (nearest_v != 0)){ref_speed = nearest_v;}
            if(v1 != -1.0 || v2 != -1.0)
            {
              if(v1<=v2){ref_speed = v1;}
              else{ref_speed = v2;}
            }
            else
            {
              ref_speed = MAX_SPEED;
            }
          

            //ref_speed = get_lane_vel(other_vehicles, t_lane);

            if(ref_speed <= 0)
            {
              ref_speed = MAX_SPEED;
            }
            */
            /*
            double min_s_diff = 99999.0;
            for(int j = 0; j < other_vehicles.size(); j++)
            {
              if(other_vehicles[j].lane == my_car.lane)
              {
                if((other_vehicles[j].s > my_car.s) && ((other_vehicles[j].s - my_car.s) < min_s_diff))
                {
                  min_s_diff = other_vehicles[j].s - my_car.s;
                }
              }
            }
            if(min_s_diff > 90){ref_speed = MAX_SPEED;}
            */
           //std::cout << ref_speed;
            //std::cout << nearest_v;
            target_speed = car_speed;
            //std::cout << "Temp state: " << temp_state << "\n";
            //std::cout << "Ref Speed: " << ref_speed << "\n";
            
            //std::cout << "CHECK POINT 0.6\n";
            //std::cout << "target_speed: " << target_speed << "\n";
            //std::cout << "ref_speed: " << ref_speed << "\n";

            while(fabs(target_speed - ref_speed) >= 0 && fabs(target_speed - car_speed) < 0.55*MAX_ACC/2.24)
            {
              

              if(fabs(target_speed - ref_speed) < 0.05)
              {
                target_speed = ref_speed;
              }
              else
              {
                if(target_speed < ref_speed){target_speed += 0.05;}
                else if(target_speed > ref_speed){target_speed -= 0.05;}
              }
              
              //std::cout << target_speed << "\n";
              
              
              if((target_speed > 0) && (target_speed <= MAX_SPEED))
            	{
                //std::cout << " target_speed:" << target_speed << "  ";
                //std::cout<<"DEBUG!";
                //std::cout<<"Before the traj_gen.\n";
            	  //vector<vector<double>> trajectory = traj_gen(end_path_d, my_car, temp_state, target_speed,
                //                                             alt_previous_path_x, 
                //                                             alt_previous_path_y, map_waypoints_s, 
                //                                             map_waypoints_x, map_waypoints_y);
                //std::cout << "CHECK POINT 1\n";

                vector<double> ptsx;
                vector<double> ptsy;

                int prev_size = alt_previous_path_x.size(); 
                double ref_yaw = deg2rad(car_yaw);
                double ref_x = car_x;
                double ref_y = car_y;

                  //if(v == 0 && i == 0){break;}

                int alt_lane = int(floor(end_path_d/4));
                if(alt_lane < 0){alt_lane = 0;}
                else if(alt_lane > 2){alt_lane = 2;}
                int target_lane;



                
                
                if(prev_size < 2)
                {
                  double prev_car_x = car_x - cos(car_yaw);
                  double prev_car_y = car_y - sin(car_yaw);

                  ptsx.push_back(prev_car_x);
                  ptsx.push_back(car_x);

                  ptsy.push_back(prev_car_y);
                  ptsy.push_back(car_y);

                  end_path_s = car_s;
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
                double target_d1, target_d2, target_d3;

                if(temp_state.compare("KL") == 0)
                {
                  target_lane = alt_lane;
                  s_plus = 30;
                  //target_d1 = 2+4*target_lane;
                  //target_d2 = 2+4*target_lane;
                  //target_d3 = 2+4*target_lane;
                }
                else if(temp_state.compare("LCL") == 0)
                {
                  target_lane = alt_lane - 1;
                  s_plus = 40;
                  //target_d1 = 4*alt_lane;
                  //target_d2 = 2+4*target_lane;
                  //target_d3 = 2+4*target_lane;
                }
                else if(temp_state.compare("LCR") == 0)
                {
                  target_lane = alt_lane + 1;
                  s_plus = 40;
                  //target_d1 = 4*target_lane;
                  //target_d2 = 2+4*target_lane;
                  //target_d3 = 2+4*target_lane;
                }

                //double ref_len = ref_vel*10;

                vector<double> next_wp0 = getXY(end_path_s+s_plus, 2+4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                vector<double> next_wp1 = getXY(end_path_s+2*s_plus, 2+4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                vector<double> next_wp2 = getXY(end_path_s+3*s_plus, 2+4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

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
                
                //std::cout << "CHECK POINT 2\n";   

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

                double N = (target_dist/(.02*target_speed/2.24));

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
                vector<vector<double>> trajectory;
                trajectory.push_back(traj_x);
                trajectory.push_back(traj_y);

                //std::cout << "CHECK POINT 3\n";
                
                /*
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
                  //std::cout << " s: " << sd[0];
                  //std::cout << " d: " << sd[1] << "\n"; 
                }

                traj_sd.push_back(traj_s);
                traj_sd.push_back(traj_d);

                vector<double> traj_vs = traj_deriv(traj_s);
                vector<double> traj_vd = traj_deriv(traj_d);
                vector<double> traj_as = traj_deriv(traj_vs);
                vector<double> traj_ad = traj_deriv(traj_vd);
                vector<double> traj_js = traj_deriv(traj_as);
                vector<double> traj_jd = traj_deriv(traj_ad);

                std::cout<<"vs: " << 
                */
                //std::cout << "CHECK POINT 3.4\n";

                //double clc = collision_cost(my_car, other_vehicles, trajectory);
                //std::cout << "The collision cost: " << clc << "\n";


                //std::cout << "CHECK POINT 3.5\n";

                //std::cout<<"After the traj_gen.\n";
                double cost = calculate_cost(my_car, other_vehicles, trajectory, map_waypoints_x, map_waypoints_y);
            	  //std::cout << "cost:" << cost << "\n";
                //std::cout << "CHECK POINT 4\n";
                //std::cout << "There are " << other_vehicles.size() << " cars.\n";
                if(cost < min_cost)
	              {
	           	  	min_cost = cost;
	              	final_traj = trajectory;
	              	final_state = temp_state;
                  //std::cout << "Temp min cost: " << min_cost << "\n";
                  //std::cout << "Temp final state: " << final_state << "\n";
	              }
            	}
            	
              if(target_speed == ref_speed){break;}
            }
          }
            
            //vector<vector<double>> trajectory = traj_gen(my_car, temp_state, ref_speed,
            //                                                 alt_previous_path_x, 
            //                                                 alt_previous_path_y, map_waypoints_s, 
            //                                                 map_waypoints_x, map_waypoints_y);
            
          std::cout << "Next state: " << final_state << "\n";
          std::cout << "Min cost: " << min_cost << "\n";

          if(state != final_state)
          {
            std::cout << "State change! New state: " << final_state << "\n";
            std::cout << "Min cost: " << min_cost << "\n";


          }
          state = final_state;
          //std::cout << "final_sate:" << final_state << "\n";
          //std::cout << "min_cost:" << min_cost << "\n";

          my_car.prev_lane = my_car.lane;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //vector<vector<double>> final_sd = xy_to_sd(my_car, final_traj, map_waypoints_x, map_waypoints_y);
          /*
          std::cout << "FINAL SD:\n" ;

          for(int i=0; i < final_sd[0].size(); i++)
          {
            std::cout << "s: " << final_sd[0][i];
            std::cout << " d: " << final_sd[1][i] << "\n";
          }
          */
          next_x_vals = final_traj[0];
          next_y_vals = final_traj[1];
          //std::cout << "CHECK POINT 5\n";

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          //std::cout << "CHECK POINT 6\n";

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}