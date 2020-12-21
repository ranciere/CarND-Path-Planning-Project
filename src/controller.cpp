#include "controller.h"
#include "helpers.h"
#include "spline.h"

void Controller::check_cars(MyData& data)
{
  // The last position of the previous path
  if (data.prev_size > 0)
  {
    data.car_s = data.end_path_s;
  }
  // Iterating over the cars
  for (int i = 0; i < data.sensor_fusion.size(); i++)
  {
    int check_car_lane = -1;
    float d = data.sensor_fusion[i][6];
    if (d > 0 && d <= 4)
    {
      // on the left lane
      check_car_lane = 0;
    }
    else if (d > 4 && d <= 8)
    {
      // on the middle lane
      check_car_lane = 1;
    }
    else if (d > 8 && d <= 12)
    {
      // on the right lane
      check_car_lane = 2;
    }
    if (check_car_lane < 0)
    {
      continue;
    }

    // check the velocity of the car
    double vx = data.sensor_fusion[i][3];
    double vy = data.sensor_fusion[i][4];
    double check_speed = sqrt(vx * vx + vy * vy);
    double check_car_s = data.sensor_fusion[i][5];
    check_car_s += (double)data.prev_size * .02 * check_speed;
    const double SAFE_CAR_DISTANCE = 20.0;

    if (check_car_lane == lane_)
    {
      if (check_car_s > data.car_s && check_car_s - data.car_s < SAFE_CAR_DISTANCE)
      {
        data.too_close = true;
      }
    }
    else if (check_car_lane == lane_ - 1)
    {
      if (check_car_s > data.car_s - SAFE_CAR_DISTANCE && check_car_s < data.car_s + SAFE_CAR_DISTANCE)
      {
        data.is_car_left = true;
      }
    }
    else if (check_car_lane == lane_ + 1)
    {
      if (check_car_s > data.car_s - SAFE_CAR_DISTANCE && check_car_s < data.car_s + SAFE_CAR_DISTANCE)
      {
        data.is_car_right = true;
      }
    }
  }
}

void Controller::logic(MyData& data)
{
  static const double MAX_SPEED = 49.5;
  static const double MAX_ACCELERATION = 2*0.224;
  static const double MAX_DECELERATION = 2*0.224;
  if (data.too_close)
  {
    if (!data.is_car_left && lane_ > 0)
    {
      lane_ -= 1;
    }
    else if (!data.is_car_right && lane_ < 2)
    {
      lane_ += 1;
    }
    else
    {
      velocity_ -= MAX_DECELERATION;
    }
  }
  else
  {
    if (velocity_ < MAX_SPEED)
    {
      velocity_ += MAX_ACCELERATION;
    }
    if (!data.is_car_left && lane_ == 2)
    {
      lane_ -= 1;
    }
    else if (!data.is_car_right && lane_ == 0)
    {
      lane_ += 1;
    }
  }
}

void Controller::calculate_trajectory(MyData& data, std::vector<double>& next_x_vals, std::vector<double>& next_y_vals)
{
  std::vector<double> ptsx;
  std::vector<double> ptsy;

  double ref_x = data.car_x;
  double ref_y = data.car_y;
  double ref_yaw = deg2rad(data.car_yaw);

  if (data.prev_size < 2)
  {
    double car_x_prev = data.car_x - cos(data.car_yaw);
    double car_y_prev = data.car_y - sin(data.car_yaw);
    ptsx.push_back(car_x_prev);
    ptsx.push_back(data.car_x);
    ptsy.push_back(car_y_prev);
    ptsy.push_back(data.car_y);
  }
  else
  {
    ref_x = data.previous_path_x[data.prev_size - 1];
    ref_y = data.previous_path_y[data.prev_size - 1];
    double ref_x_prev = data.previous_path_x[data.prev_size - 2];
    double ref_y_prev = data.previous_path_y[data.prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  auto next_wp0 = getXY(data.car_s + 30, 2 + 4 * lane_, map_s_, map_x_, map_y_);
  auto next_wp1 = getXY(data.car_s + 60, 2 + 4 * lane_, map_s_, map_x_, map_y_);
  auto next_wp2 = getXY(data.car_s + 90, 2 + 4 * lane_, map_s_, map_x_, map_y_);
  ptsx.push_back(std::get<0>(next_wp0));
  ptsx.push_back(std::get<0>(next_wp1));
  ptsx.push_back(std::get<0>(next_wp2));
  
  ptsy.push_back(std::get<1>(next_wp0));
  ptsy.push_back(std::get<1>(next_wp1));
  ptsy.push_back(std::get<1>(next_wp2));

  // convert to local car coordinate
  for (int i = 0; i < ptsx.size(); i++)
  {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
    ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
  }

  // spline
  tk::spline s;
  s.set_points(ptsx, ptsy);

  for (int i = 0; i < data.prev_size; i++)
  {
    next_x_vals.push_back(data.previous_path_x[i]);
    next_y_vals.push_back(data.previous_path_y[i]);
  }

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  double x_add_on = 0;

  for (int i = 1; i <= 50 - data.prev_size; i++)
  {
    double N = target_dist / (.02 * velocity_ / 2.24);
    double x_point = x_add_on + target_dist / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}

void Controller::update(const Data &data_, std::vector<double> &next_x_vals, std::vector<double> &next_y_vals)
{
  MyData data(data_);

  check_cars(data);
  logic(data);
  calculate_trajectory(data, next_x_vals, next_y_vals);
}
