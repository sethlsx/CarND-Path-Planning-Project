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
#include "trajectory_generate.h" //Hide the trajectory generate code in a head file

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

          vector<vector<double>> alt_sensor_fusion;

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
             alt_sensor_fusion.push_back(sensor_fusion[i]);
          }

          

          vector<string> available_states;
          double min_cost = 99999999.0;
          vector<vector<double>> final_traj;
          //string state = my_car.state;
          int lane = my_car.lane;

          if(state.compare("KL") == 0) 
          {
            available_states.push_back("KL");
            if(lane > 0){available_states.push_back("LCL");}
            if(lane < 2){available_states.push_back("LCR");}
          } else if (state.compare("LCL") == 0) {
            available_states.push_back("KL");
            if(lane > 0){available_states.push_back("LCL");}
          } else if (state.compare("LCR") == 0) {
            available_states.push_back("KL");
            if(lane < 2){available_states.push_back("LCR");}
          }




          //available_states = my_car.successor_states();
          string final_state;
          string temp_state;

          double target_speed;
          int target_lane;

          for(int i = 0; i < available_states.size(); i++)
          {
            double ref_speed = MAX_SPEED;
            if(other_vehicles.size() != 0)
            {
              temp_state = available_states[i];
              //std::cout << temp_state << ":\n";
              if(temp_state.compare("KL") == 0)
              {
                target_lane = my_car.lane;
              }
              else if(temp_state.compare("LCL") == 0)
              {
                target_lane = my_car.lane - 1;
              }
              else if(temp_state.compare("LCR") == 0)
              {
                target_lane = my_car.lane + 1;
              }


              double min_s = 9999999;
              double nearest_v;
              for(int j = 0; j < other_vehicles.size(); j++)
              {
                if(other_vehicles[j].lane == target_lane)
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
              if(min_s < 30){ref_speed = nearest_v;}
            }
            
            if(ref_speed == 0){
              ref_speed = MAX_SPEED;
            }
           //std::cout << ref_speed;
            //std::cout << nearest_v;
            target_speed = car_speed;
            while(fabs(target_speed - ref_speed) > 0.2 && fabs(target_speed - car_speed)/0.02 < MAX_ACC)
            {
              if(target_speed < ref_speed){target_speed += 0.1;}
              else if(target_speed > ref_speed){target_speed -= 0.1;}
              //std::cout << target_speed << "\n";
              
              
              if((target_speed > 0) && (target_speed <= MAX_SPEED))
            	{
                //std::cout << " target_speed:" << target_speed << "  ";
                //std::cout<<"DEBUG!";
            	  vector<vector<double>> trajectory = traj_gen(car_x, car_y, car_yaw, temp_state, target_speed,
                                                             lane, car_s,
                                                             alt_previous_path_x, 
                                                             alt_previous_path_y, map_waypoints_s, 
                                                             map_waypoints_x, map_waypoints_y);
                
                double cost = calculate_cost(my_car, other_vehicles, trajectory, map_waypoints_x, map_waypoints_y);
            	  //std::cout << "cost:" << cost << "\n";
                if(cost <= min_cost)
	              {
	           	  	min_cost = cost;
	              	final_traj = trajectory;
	              	final_state = temp_state;
	              }
            	}
            	
            }
            
           }

          state = final_state;
          std::cout << "final_sate:" << final_state << "\n";
          std::cout << "min_cost:" << min_cost << "\n";

          my_car.prev_lane = my_car.lane;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          
          
          next_x_vals = final_traj[0];
          next_y_vals = final_traj[1];


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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