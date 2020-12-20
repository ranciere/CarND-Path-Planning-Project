#pragma once

#include <vector>
#include "json.hpp"

struct Data
{
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;

  // Previous path data given to the Planner
  std::vector<double> previous_path_x;
  std::vector<double> previous_path_y;
  // Previous path's end s and d values
  double end_path_s;
  double end_path_d;

  // Sensor Fusion Data, a list of all other cars on the same side
  //   of the road.
  std::vector<std::vector<double>> sensor_fusion;

  static Data extract_from_json(const nlohmann::json& json)
  {
    Data data;
    data.car_x = json["x"];
    data.car_y = json["y"];
    data.car_s = json["s"];
    data.car_d = json["d"];
    data.car_yaw = json["yaw"];
    data.car_speed = json["speed"];

    // Previous path data given to the Planner
    data.previous_path_x = json["previous_path_x"].get<std::vector<double>>();
    data.previous_path_y = json["previous_path_y"].get<std::vector<double>>();
    // Previous path's end s and d values
    data.end_path_s = json["end_path_s"];
    data.end_path_d = json["end_path_d"];

    // Sensor Fusion Data, a list of all other cars on the same side
    //   of the road.
    data.sensor_fusion = json["sensor_fusion"].get<std::vector<std::vector<double>>>();
    return data;
  }
};

