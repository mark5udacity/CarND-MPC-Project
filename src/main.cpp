#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

using json = nlohmann::json;

// CONSTANTs
static const int WEBSOCKECT_OK_DISCONNECT_CODE = 1000;
static const std::string RESET_SIMULATOR_WS_MESSAGE = "42[\"reset\", {}]";
static const std::string MANUAL_WS_MESSAGE = "42[\"manual\",{}]";

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

void sendMessage(uWS::WebSocket<uWS::SERVER> ws, std::string msg) {
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  bool firstTimeConnecting = true;
  bool justSwitchedToManual = true;
  h.onMessage([&mpc, &justSwitchedToManual](uWS::WebSocket<uWS::SERVER> ws,
                                            char *data,
                                            size_t length,
                                            uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          if (!justSwitchedToManual) {
            std::cout << "Taking back control from manual!!" << std::endl;
            justSwitchedToManual = true;
          }

          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value;
          double throttle_value;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          sendMessage(ws, msg);
        }
      } else {
        if (justSwitchedToManual) {
            justSwitchedToManual = false;
            std::cout << "Switched to manual mode!!" << std::endl;
        }

        sendMessage(ws, MANUAL_WS_MESSAGE);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res,
                     uWS::HttpRequest req,
                     char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h, &firstTimeConnecting](uWS::WebSocket<uWS::SERVER> ws,
                                            uWS::HttpRequest req) {
      if (firstTimeConnecting) {
          std::cout << "Connected for first time!!!  Restarting simulator!" << std::endl;
          firstTimeConnecting = false;
          sendMessage(ws, RESET_SIMULATOR_WS_MESSAGE);
      } else {
          std::cout << "Reconnected to simulator, success!" << std::endl;
      }
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws,
                         int code,
                         char *message,
                         size_t length) {
      if (code == WEBSOCKECT_OK_DISCONNECT_CODE) {
        std::cout << "Disconnected normally." << std::endl;
      } else {
        std::cout << "Unexpected Disconnect with code: " << code << "!" << std::endl;
      }

      std::cout << "WARN: Not closing WS because we get bad access exception! (Which one cannot catch in C++!?)"  << std::endl;
      // StackOverflow https://stackoverflow.com/q/19304157 suggested that error code 1006 means to check onError
      // But, but, but, adding onError here doesn't get called at all, everything seems alright
      // (other than this ws.close() exc_bad_access)
      // ws.close(code, message, length);
      // Besides ^^, if we're getting disconnection method, then the connection is already closed??
  });

  h.onError([](void *user) {
      // Code copied from: https://github.com/uNetworking/uWebSockets/blob/master/tests/main.cpp
      switch ((long) user) {
        case 1:
          std::cout << "Client emitted error on invalid URI" << std::endl;
              break;
        case 2:
          std::cout << "Client emitted error on resolve failure" << std::endl;
              break;
        case 3:
          std::cout << "Client emitted error on connection timeout (non-SSL)" << std::endl;
              break;
        case 5:
          std::cout << "Client emitted error on connection timeout (SSL)" << std::endl;
              break;
        case 6:
          std::cout << "Client emitted error on HTTP response without upgrade (non-SSL)" << std::endl;
              break;
        case 7:
          std::cout << "Client emitted error on HTTP response without upgrade (SSL)" << std::endl;
              break;
        case 10:
          std::cout << "Client emitted error on poll error" << std::endl;
              break;
        case 11:
          static int protocolErrorCount = 0;
              protocolErrorCount++;
              std::cout << "Client emitted error on invalid protocol" << std::endl;
              if (protocolErrorCount > 1) {
                std::cout << "FAILURE:  " << protocolErrorCount << " errors emitted for one connection!" << std::endl;
              }
              break;
        default:
          std::cout << "FAILURE: " << user << " should not emit error!" << std::endl;
      }
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
