#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if(found_null != std::string::npos) {
    return "";
  }else if(b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  pid.Init(-0.083349, -0.000382, -0.783738);
  const int lapstep = 650;
  static double p[] = {-0.083349, -0.000382, -0.783738};
  static double dp[] = {-0.002, -0.00002, -0.02};
  static int step = 0;
  static double accErr = 0, bestErr = 1e100;
  static bool fliped = false;
  static int optimize_index = 0;

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if(length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if(s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();

        if(step++ > lapstep) {
          step = 0;
          accErr /= lapstep;
          if(accErr < bestErr) {
            bestErr = accErr;
            dp[optimize_index % 3] *= 1.1;
            ++optimize_index;
            p[optimize_index % 3] += dp[optimize_index % 3];
            fliped = false;
          }else {
            if(fliped) {
              p[optimize_index % 3] -= dp[optimize_index % 3];
              dp[optimize_index++ % 3] *= 0.9;
              p[optimize_index % 3] += dp[optimize_index % 3];
              fliped = false;
            }else {
              dp[optimize_index % 3] = -dp[optimize_index % 3];
              p[optimize_index % 3] += 2 * dp[optimize_index % 3];
              fliped = true;
            }
          }
          pid.twi(p);
          accErr = 0;
          printf("\n%f %f %f: %f \t %f %f %f", p[0], p[1], p[2], bestErr, dp[0], dp[1], dp[2]);
          fflush(stdout);
        }


        if(event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          accErr += fabs(cte);

          // DEBUG
//          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
