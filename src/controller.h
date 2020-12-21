#pragma once
#include <vector>
#include "data.h"

class Controller
{
public:
  using wp_t = std::vector<double>;
private:
  struct MyData: Data
  {
    int prev_size = 0;
    bool too_close = false;
    bool is_car_left = false;
    bool is_car_right = false;
    MyData(Data d): Data(d), prev_size(d.previous_path_x.size()) {}
  };

  // Current lane
  int lane_ = 1;
  // Current speed
  double velocity_ = 0.0;
  // Map data 
  wp_t map_x_;
  wp_t map_y_;
  wp_t map_s_;
  wp_t map_dx_;
  wp_t map_dy_;
  //// Steps of the algorithm
  // Check surrounding cars
  void check_cars(MyData& data);
  // Calculate lanes
  void logic(MyData& data);
  // Calcluate trajectory
  void calculate_trajectory(MyData& data, std::vector<double>& next_x_vals, std::vector<double>& next_y_vals);
public:
  Controller(wp_t x, wp_t y, wp_t s, wp_t dx, wp_t dy): map_x_(x), map_y_(y), map_s_(s), map_dx_(dx), map_dy_(dy) {}
  void update(const Data& data, std::vector<double>& next_x_vals, std::vector<double>& next_y_vals);
};