#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <limits>

// for convenience
using json = nlohmann::json;

// best tau values (manually selected, based on comfort feeling)
double K_P = 0.12;
double K_I = 0.0015;
double K_D = 5.0;

// calibrated tau values, based on criteria of minimal mean squared error: [0.245, 0.0008949, 8.74359]

constexpr double MAX_CALIBRATION_STEPS = 2000;

// calibration variables
double delta_tau_tolerance = 0.0;
double delta_tau = 0.0;
double squared_eror_sum = 0.0; // sum of suared errors
int calibration_steps_counter = 0;
double best_mse = std::numeric_limits<double>::max();
double current_param_value = K_P;
double best_param_value = 0.0;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}


void ResetSimulator(uWS::WebSocket<uWS::SERVER> ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
  calibration_steps_counter = 0;
  squared_eror_sum = 0.0; // mean squared error
}

void CalibrateParam(const double mse, double& param_value, double& delta, bool& reset_flag){
 if(delta > delta_tau_tolerance) {

    if (reset_flag) { // param value was not yet updated
       current_param_value = param_value;
       param_value+=delta;
       std::cout << "Param increased from : " << current_param_value << " to " << param_value << std::endl;
       reset_flag = false;

    } else {

        if (mse < best_mse) {
            best_mse = mse;
            best_param_value = param_value; // store best value of parameter as a workaround for 'local optimum' issue
            delta*=1.1;
            std::cout << "Delta increased by 1.1" << " to " << delta << std::endl;
            reset_flag = true;
            CalibrateParam(mse, param_value, delta_tau, reset_flag);

        } else {
            if (current_param_value < param_value) {
                //param value was increased by d_param during previous calibration step but this was not efficent
                current_param_value = param_value;
                param_value-= 2.0 * delta; // decrease to previous value and try by d_param less
                std::cout << "Param decreased from : " << current_param_value << " to " << param_value << std::endl;
                reset_flag = false;
            } else {
                // seems like param value was decreased in previous step and this was not efficient
                current_param_value = param_value;
                param_value+= delta_tau; // restore param value to initial state
                delta*= 0.9; // decrease d_param
                reset_flag = true;
                std::cout << "Delta decreased by 0.9" << " to " << delta << std::endl;
                std::cout << "Param restored to " << param_value << std::endl;
            }
        }
    }
  } else {
    std::cout << "Calibration finished!: Best MSE: " << best_mse << " with PID parameter value " << best_param_value << std::endl;
    exit(0);
  }
}

int main(int argC, char** argV)
{
  bool in_caliration_mode = false;
  bool calibration_reset = true;
  double* calibration_param = nullptr;
  PID pid;

  // check if PID controller has to run in  calibration mode to find best values for tau parameters
  if (argC > 2) {
    std::string mode = argV[1];
    if (mode.compare("calibrate") == 0) {
      in_caliration_mode = true;
    }
    std::string calibration_param_name = argV[2];
    if (calibration_param_name.compare("P")==0) {
      calibration_param = &pid.Kp_;
      // calibration process settings
      K_P = 0.05;
      delta_tau_tolerance = 0.005;
      delta_tau = 0.01;
    } else if (calibration_param_name.compare("I")==0){
      calibration_param = &pid.Ki_;
      // calibration process settings
      K_I = 0.0005;
      delta_tau_tolerance = 0.00005;
      delta_tau = 0.0001;
    } else if (calibration_param_name.compare("D")==0) {
      K_D = 3.0;
      calibration_param = &pid.Kd_;
      // calibration process settings
      K_D = 3.0;
      delta_tau_tolerance = 0.2;
      delta_tau = 0.5;
    } else {
      std::cerr << "Unsupported PID param name " << calibration_param << std::endl;
      return -1;
    }
  }

  uWS::Hub h;


  // Initialize the PID controller with default tau values
  pid.Init(K_P, K_I, K_D);

  h.onMessage([&pid, &in_caliration_mode, &calibration_reset, &calibration_param](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * Calcuate steering value in range [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          // Update error values with new cte
          pid.UpdateError(cte);

          // Calculate value for steer controller within allowed range [-1, 1]
          steer_value = std::min(1.0, std::max(-1.0, pid.TotalError()));

          // DEBUG

          if (in_caliration_mode) {
            calibration_steps_counter++;

            if (calibration_steps_counter > MAX_CALIBRATION_STEPS) {
                // we gathered enough data for calibration, so can calibrate now
                const double mse = squared_eror_sum / (calibration_steps_counter-1);
                std::cout << " Best MSE: " << best_mse << " Current MSE: " << mse << " with PID params: " <<  pid.Kp_ << ": " <<  pid.Ki_ << " : " <<  pid.Kd_ << std::endl;
                CalibrateParam(mse, *calibration_param, delta_tau, calibration_reset);
                ResetSimulator(ws);
                return;
            } else {
              // update mean squared error
              squared_eror_sum+= pow(cte, 2);
            }

          } else {
              std::cout << " Steering Angle: " << angle << ", CTE: " << cte << ", Calculated steer Value: " << steer_value << std::endl;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;

          // Throttle value must correlate with steering angle.
          // means the bigger the steering angle is in both left and right direction in range [0:1],
          // smaller the throttle must be (even lowering to negative value for slight braking).
          double throttle = 0.7 - std::abs(cte)/4 - std::abs(steer_value);
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
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
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
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
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
