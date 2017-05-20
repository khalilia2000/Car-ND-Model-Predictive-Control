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

// for convenience
using json = nlohmann::json;

// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

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
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
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

int main() {

  const int latency_dt = 100; // in milliseconds
  
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
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
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value = 0;
          double throttle_value = 0;

          // step 1 - convert coordinates of ptsx and ptsy to local vehicle coordinates
          Eigen::VectorXd ptsx_vcoord(ptsx.size());
          Eigen::VectorXd ptsy_vcoord(ptsy.size());
          for (int i=0; i<ptsx.size(); i++) {
            ptsx_vcoord(i) =  (ptsx[i] - px)*cos(psi) + (ptsy[i] - py)*sin(psi);
            ptsy_vcoord(i) = -(ptsx[i] - px)*sin(psi) + (ptsy[i] - py)*cos(psi);
          }
          
          // step 2 - calculate coeffs
          auto coeffs = polyfit(ptsx_vcoord, ptsy_vcoord, 3);
          cout << "coeffs = " << coeffs << endl;
          
          // step 3 - claculate the state for time 0 using vehicle coordintes
          double x0 = 0;
          double y0 = 0;
          double psi0 = 0;
          double v0 = v;
          double cte0 = y0 - polyeval(coeffs, x0);
          double psides0 = atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*x0*x0);
          double epsi0 = psi0 - psides0;

          // step 4 - calculate state at the end of the latency using vehicle coordinates
          double x1 = x0 + v0*cos(psi0)*latency_dt/1000.0;
          double y1 = y0 + v0*sin(psi0)*latency_dt/1000.0;
          double psi1 = psi0 + v0*steer_value/Lf*latency_dt/1000.0;
          double v1 = v0 + throttle_value*latency_dt/1000.0;
          double cte1 = y1 - polyeval(coeffs, x1);
          double epsi1 = (epsi0 + (v0*steer_value/Lf*latency_dt/1000.0));

          // step 5 - form state variable using end of latency
          Eigen::VectorXd state(6);
          state << x1, y1, psi1, v1, cte1, epsi1;

          // step 6 - solve for the actuations
          auto vars = mpc.Solve(state, coeffs);
          
          // set actuation
          steer_value = -vars[vars.size()-2];
          throttle_value = vars[vars.size()-1];

          // setting up the json message
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          int x_start = 0;
          int y_start = (vars.size()-2)/2;
          for (int i=0; i<(vars.size()-2)/2; i++) {
            mpc_x_vals.push_back(vars[x_start+i]);
            mpc_y_vals.push_back(vars[y_start+i]);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for (int i=0; i<ptsx_vcoord.size(); i++) {
            next_x_vals.push_back(ptsx_vcoord(i));
            next_y_vals.push_back(ptsy_vcoord(i));
          }

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
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(latency_dt));
          (*ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        (*ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code,
                         char *message, size_t length) {
    (*ws).close();
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
