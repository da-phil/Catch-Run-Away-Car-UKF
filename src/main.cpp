#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include <getopt.h>
#include <cstdlib>

#include "ukf.h"
#include "ekf.h"

using namespace std;

// for convenience
using json = nlohmann::json;


bool verbose = false;
bool use_laser = true;
bool use_radar = true;
string filter_choice  = "ukf";
// The following UKF process noise values achieve an RMSE of 
// [0.0638, 0.084, 0.332, 0.217] in px, py, vx, vy.
double std_a = 0.6;
double std_yawdd = 0.4;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}


MeasurementPackage getMeasurement(string &sensor_measurement)
{
  MeasurementPackage meas_package;
  istringstream iss(sensor_measurement);
  long long timestamp;
  float px, py, ro, theta, ro_dot, x_gt, y_gt, vx_gt, vy_gt, yaw_gt, yawrate_gt;

  // reads first element from the current line
  string sensor_type;
  iss >> sensor_type;
  meas_package.ground_truth_ = VectorXd(6);

  if (sensor_type.compare("L") == 0) {
        meas_package.sensor_type_ = MeasurementPackage::LASER;
        meas_package.raw_measurements_ = VectorXd(2);
        iss >> px;
        iss >> py;
        meas_package.raw_measurements_ << px, py;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
  } else if (sensor_type.compare("R") == 0) {
        meas_package.sensor_type_ = MeasurementPackage::RADAR;
        meas_package.raw_measurements_ = VectorXd(3);
        iss >> ro;
        iss >> theta;
        iss >> ro_dot;
        meas_package.raw_measurements_ << ro,theta, ro_dot;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
  }

  /*
  iss >> x_gt;
  iss >> y_gt;
  iss >> vx_gt;
  iss >> vy_gt;
  iss >> yaw_gt;
  iss >> yawrate_gt;
  meas_package.ground_truth_ << x_gt, y_gt, vx_gt, vy_gt, yaw_gt, yawrate_gt;
  */
  return meas_package;
}



void PrintHelp() {
    std::cout <<
            "Help:\n"
            "  --filter         <ekf|ukf>:  Choose between EKF and UKF, default: "<<filter_choice<<"\n"
            "  --verbose        <0|1>:      Turn on verbose output, default: "<<verbose<<"\n"
            "  --use_laser      <0|1>:      Turn on or off laser measurements, default: "<<use_laser<<"\n"
            "  --use_radar      <0|1>:      Turn on or off radar measurements, default: "<<use_radar<<"\n"
            "  --std_a          <num>:      Standard deviation for linear acceleration noise, default: "<<std_a<<"\n"
            "  --std_yawdd      <num>:      Standard deviation for angular acceleration noise, default: "<<std_yawdd<<"\n"
            "  --help:                      Show help\n";
    exit(1);
}



void ParseArgs(int argc, char *argv[])
{
  char c;
  const char* const short_opts = "v:l:r:a:y:h";
  const option long_opts[] = {
          {"verbose",       1, nullptr, 'v'},
          {"use_laser",     1, nullptr, 'l'},
          {"use_radar",     1, nullptr, 'r'},
          {"std_a",         1, nullptr, 'a'},
          {"std_yawdd",     1, nullptr, 'y'},
          {"filter",        1, nullptr, 'f'},           
          {"help",          0, nullptr, 'h'},
          {nullptr,         0, nullptr, 0}
  };

  while (true)
  {
    const auto opt = getopt_long(argc, argv, short_opts, long_opts, nullptr);

    if (opt == -1)
        break;

    switch (opt)
    {
      case 'v':
        verbose = (stoi(optarg) > 0) ? true : false;
        break;
      case 'l':
        use_laser = (stoi(optarg) > 0) ? true : false;
        break;
      case 'r':
        use_radar = (stoi(optarg) > 0) ? true : false;
        break;
      case 'a':
        std_a = atof(optarg);
        break;
      case 'y':
        std_yawdd = atof(optarg);
        break;
      case 'f':
        filter_choice = string(optarg);
        break;
      case 'h':
      default:
        PrintHelp();
        break;
    }
  }
}



int main(int argc, char *argv[])
{
  uWS::Hub h;

  // Parse cmd args
  ParseArgs(argc, argv);
    cout << "========== Filter config ==========" << endl << "filter_choice="<< filter_choice << ", use_laser="<<use_laser<< ", use_radar="<<use_radar <<
          ", verbose="<<verbose << ", std_a="<<std_a << ", std_yawdd="<<std_yawdd << endl;

  // Create a generic filter
  Filter* filter;
  
  if (filter_choice.compare("ukf") == 0) {
    filter = new UKF(verbose, use_laser, use_radar, std_a, std_yawdd);
  } else {
    filter = new EKF(verbose, use_laser, use_radar, std_a, std_yawdd);
  }
  
  double target_x = 0.0;
  double target_y = 0.0;
  long long last_timestamp = 0.0;

  h.onMessage([filter,&target_x,&target_y,&last_timestamp](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(std::string(data));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double target_yaw = 0.0;
          double target_yawd = 0.0;
          double target_v = 0.0;
          double target_x_pred = 0.0;
          double target_y_pred = 0.0;
          double target_yaw_pred = 0.0;
          double dt;

          double hunter_x = std::stod(j[1]["hunter_x"].get<std::string>());
          double hunter_y = std::stod(j[1]["hunter_y"].get<std::string>());
          double hunter_heading = std::stod(j[1]["hunter_heading"].get<std::string>());
          
          string lidar_measurement = j[1]["lidar_measurement"];
          string radar_measurement  = j[1]["radar_measurement"];
          MeasurementPackage lidar_meas_package = getMeasurement(lidar_measurement);
          filter->ProcessMeasurement(lidar_meas_package);
          MeasurementPackage radar_meas_package = getMeasurement(radar_measurement);
          filter->ProcessMeasurement(radar_meas_package);

          dt = (lidar_meas_package.timestamp_ - last_timestamp) / 1.0e6;
          //cout << "dt: " << dt << endl;
          last_timestamp = lidar_meas_package.timestamp_;
          target_x = filter->x_[0];
          target_y = filter->x_[1];
          target_v = filter->x_[2];
          target_yaw = filter->x_[3];
          target_yawd = filter->x_[4];

          // predict where run away car will be in next timestep!
          target_x_pred = target_x + 50*dt*target_v*cos(target_yaw);
          target_y_pred = target_y + 50*dt*target_v*sin(target_yaw);
          target_yaw_pred = fmod(target_yaw + dt*target_yawd, 2.*M_PI);

          double heading_to_target = fmod(atan2(target_y_pred - hunter_y, target_x_pred - hunter_x), 2.*M_PI);
          //turn towards the target
          double heading_difference = fmod((heading_to_target - hunter_heading), 2.*M_PI);
          double distance_difference = sqrt((target_y_pred - hunter_y)*(target_y_pred - hunter_y) + (target_x_pred - hunter_x)*(target_x_pred - hunter_x));

          json msgJson;
          msgJson["turn"] = heading_difference;
          msgJson["dist"] = distance_difference; 
          auto msg = "42[\"move_hunter\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
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























































































