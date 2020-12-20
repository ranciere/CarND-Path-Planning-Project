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

  int lane = 1;
  double ref_vel = 0.0;

  wp_t map_x;
  wp_t map_y;
  wp_t map_s;
  wp_t map_dx;
  wp_t map_dy;

  void check_cars(MyData& data);
  void logic(MyData& data);
  void calculate_trajectory(MyData& data, std::vector<double>& next_x_vals, std::vector<double>& next_y_vals);
public:
  Controller(wp_t x, wp_t y, wp_t s, wp_t dx, wp_t dy): map_x(x), map_y(y), map_s(s), map_dx(dx), map_dy(dy) {}
  void update(const Data& data, std::vector<double>& next_x_vals, std::vector<double>& next_y_vals);
};