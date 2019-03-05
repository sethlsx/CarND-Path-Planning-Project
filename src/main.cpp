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
//#include "vehicle.h"
// #include "fsm.h" //Finite State Machines for Behavior Planning
#include "trajectory_generate.h" //Hide the trajectory generate code in a head file

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;
  std::cout << "DEBUG";
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

          
          
          int lane = int(floor(car_d/4));
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

          //vector<Vehicle> other_vehicles;
          
          

          for(int i = 0; i < sensor_fusion.size(); i++)
          {
             //d = sensor_fusion[i][6];
             //lane = int(floor(d/4));
             //s = sensor_fusion[i][5];
             //vector<double> xy;
             //xy = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
             //x = xy[0];
             //y = xy[1];
             //double vx = sensor_fusion[i][3];
             //double vy = sensor_fusion[i][4];
             //v = sqrt(vx*vx+vy*vy);

             //Vehicle other_vehicle = Vehicle(lane, x, y, d, s, v, vx, vy, yaw);
             //other_vehicles.push_back(other_vehicle);
             alt_sensor_fusion.push_back(sensor_fusion[i]);
          }

          

          vector<string> available_states;
          double min_cost = 99999999.0;
          double cost;
          vector<vector<double>> final_traj;

          if(state.compare("KL") == 0) 
          {
            available_states.push_back("KL");
            if(lane > 0){available_states.push_back("LCL");}
            if(lane < 2){available_states.push_back("LCR");}
          } else if (state.compare("LCL") == 0) {
            available_states.push_back("KL");
          } else if (state.compare("LCR") == 0) {
            available_states.push_back("KL");
          }

          //available_states = ego_car.successor_states();
          string final_state;
          string temp_state;



          for(int i = 0; i < available_states.size(); i++)
          {
            temp_state = available_states[i];
            std::cout << temp_state;
            vector<vector<vector<double>>> trajectories = traj_gen(car_x, car_y, car_yaw, end_path_s, 
                                                                   temp_state, lane, car_speed, 
                                                                   alt_previous_path_x, 
                                                                   alt_previous_path_y, map_waypoints_s, 
                                                                   map_waypoints_x, map_waypoints_y);
            for(int j = 0; j < trajectories.size(); j++)
            {
              vector<vector<double>> trajectory = trajectories[j];
              cost = calculate_cost(alt_sensor_fusion, trajectory, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              if(cost < min_cost)
              {
                min_cost = cost;
                final_traj = trajectory;
                final_state = temp_state;
              }
            }
            
          }

          state = final_state;
          std::cout << final_state;
          std::cout << min_cost;

              
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     

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