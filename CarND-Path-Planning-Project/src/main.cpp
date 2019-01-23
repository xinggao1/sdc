#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"
#include "spline.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if(found_null != string::npos) {
    return "";
  }else if(b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if(dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }
  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {

  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = min(2 * pi() - angle, angle);

  if(angle > pi() / 4) {
    closestWaypoint++;
    if(closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }
  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if(next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if(centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double>
getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
  int prev_wp = -1;

  while(s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

int main() {
  uWS::Hub h;

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
  double dt = 0.02;//20ms
  const double speedLimit = 50;
  int selected_lane = 1;
  bool lane_change = false;
  double laneChangeDist = 30;
  double slowDist = 10;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while(getline(in_map_, line)) {
    istringstream iss(line);
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

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, dt,
              &selected_lane, &lane_change, slowDist, laneChangeDist, speedLimit](
  uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
  uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if(length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if(s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if(event == "telemetry") {
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

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double pos_x, pos_y, angle;
          int prePath_size = previous_path_x.size();
          int usePre_size = min(25, prePath_size);
          for(int i = 0; i < usePre_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }


          bool slowdown = false, considerLaneChange = false;
          vector<int> laneMin = {INT_MAX, INT_MAX, INT_MAX};
          for(auto &car1: sensor_fusion) {
            double car1_vx = car1[3], car1_vy = car1[4], car1_s = car1[5], car1_d = car1[6];
            double car1_v = sqrt(car1_vx * car1_vx + car1_vy * car1_vy);
            if(abs(car1_d - end_path_d) < 2 && car1_s > car_s && car1_s - end_path_s < laneChangeDist) {
              considerLaneChange = true;
              if(car1_s - end_path_s < slowDist && car1_v < car_speed)
                slowdown = true;
            }
            for(int i = 0; i < 3; ++i) {
              double d_i = 2 + 4 * i;
              if(abs(car1_d - d_i) < 2) {
                if(car1_s > car_s - 5) {
                  if(car1_s - end_path_s < slowDist)
                    laneMin[i] = min(0, laneMin[i]);
                  else
                    laneMin[i] = min(static_cast<int>(car1_s - end_path_s), laneMin[i]);
                }else if(car1_s > car_s - slowDist && car1_v > car_speed + 5) {
                  laneMin[i] = min(0, laneMin[i]);
                }
              }
            }
          }
          if(slowdown) {
            car_speed -= 2;
            cout << car_speed << endl;
            fflush(stdout);
            next_y_vals.resize(min(usePre_size, 10));
            next_x_vals.resize(min(usePre_size, 10));
          }else if(car_speed < speedLimit - 2 && laneMin[selected_lane] > slowDist * 2) {
            cout << car_speed << '+' << speedLimit - car_speed - 0.5 << endl;
            fflush(stdout);
            car_speed += min(2.0, speedLimit - car_speed - 0.5);
            next_x_vals.resize(min(usePre_size, 10));
            next_y_vals.resize(min(usePre_size, 10));
          }
//          considerLaneChange = 1;
          vector<int> laneIdx = {0, 1, 2};
          sort(laneIdx.begin(), laneIdx.end(), [&](int i1, int i2) { return laneMin[i1] > laneMin[i2]; });
          if(considerLaneChange & !lane_change) {
            for(int i = 0; i < 3; ++i)
              cout<<laneMin[i]<<'\t';
            cout<<endl;
            for(int i = 0; i < 3; ++i) {
              if(laneMin[laneIdx[i]] > laneChangeDist + 5) {
                if(abs(laneIdx[i] - selected_lane) == 1) {
                  printf("dist: %d %d %d, idx order: %d %d %d, %d -> %d\n", laneMin[0], laneMin[1],
                         laneMin[2], laneIdx[0], laneIdx[1], laneIdx[2], selected_lane, laneIdx[i]);
                  fflush(stdout);
                  selected_lane = laneIdx[i];
                  lane_change = true;
                  break;
                }else if(i == 0 && abs(laneIdx[i] - selected_lane) == 2 &&
                         laneMin[(laneIdx[i] + selected_lane) / 2] > 0) {
                  int midlane = (laneIdx[i] + selected_lane) / 2;
                  printf("dist: %d %d %d, idx order: %d %d %d, %d -> %d to midlane\n", laneMin[0], laneMin[1],
                         laneMin[2], laneIdx[0], laneIdx[1], laneIdx[2], selected_lane, midlane);
                  fflush(stdout);
                  selected_lane = midlane;
                  lane_change = true;
                  break;
                }
              }
            }
          }
          int extra_s = 0;
          if(lane_change) {
            extra_s = 20;
            double d_i = 2 + 4 * selected_lane;
            if(abs(d_i - car_d) < 0.5)
              lane_change = false;
          }


          vector<double> ptsx, ptsy;
          if(usePre_size < 2) {
            pos_x = car_x;
            pos_y = car_y;
            angle = deg2rad(car_yaw);
            double pre_x = car_x - cos(angle);
            double pre_y = car_y - sin(angle);
            ptsx = {pre_x, pos_x};
            ptsy = {pre_y, pos_y};
          }else {
            pos_x = next_x_vals[next_x_vals.size() - 1];
            pos_y = next_y_vals[next_y_vals.size() - 1];
            double pre_x = next_x_vals[next_x_vals.size() - 2];
            double pre_y = next_y_vals[next_y_vals.size() - 2];
            angle = atan2(pos_y - pre_y, pos_x - pre_x);
            ptsx = {pre_x, pos_x};
            ptsy = {pre_y, pos_y};
          }

          auto nextwp30 = getXY(car_s + 30 + extra_s, 2 + 4 * selected_lane, map_waypoints_s, map_waypoints_x,
                                map_waypoints_y);
          auto nextwp60 = getXY(car_s + 60 + extra_s, 2 + 4 * selected_lane, map_waypoints_s, map_waypoints_x,
                                map_waypoints_y);
          auto nextwp90 = getXY(car_s + 90 + extra_s, 2 + 4 * selected_lane, map_waypoints_s, map_waypoints_x,
                                map_waypoints_y);
          ptsx.insert(ptsx.end(), {nextwp30[0], nextwp60[0], nextwp90[0]});
          ptsy.insert(ptsy.end(), {nextwp30[1], nextwp60[1], nextwp90[1]});

          for(int i = 0; i < ptsx.size(); ++i) {
            double shiftx = ptsx[i] - pos_x;
            double shifty = ptsy[i] - pos_y;
            ptsx[i] = shiftx * cos(-angle) - shifty * sin(-angle);
            ptsy[i] = shiftx * sin(-angle) + shifty * cos(-angle);
          }

          tk::spline s;
          s.set_points(ptsx, ptsy);
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = distance(0, 0, target_x, target_y);
          double N = target_dist / (car_speed / 2.24 * dt);

          usePre_size = next_x_vals.size();
          for(int i = 1; i < 50 - usePre_size; i++) {
            double refx = target_x / N * i;
            double refy = s(refx);
            double x = refx * cos(angle) - refy * sin(angle) + pos_x;
            double y = refx * sin(angle) + refy * cos(angle) + pos_y;
            next_x_vals.push_back(x);
            next_y_vals.push_back(y);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      }else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if(req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    }else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if(h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  }else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
