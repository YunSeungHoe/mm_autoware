// Copyright (c) 2018-2020 Intel Corporatio
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT

#ifndef VEHICLE_PID_CONTROLLER_HPP_
#define VEHICLE_PID_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <utility>
#include <vector>

enum PIDGain {
  K_P = 0,
  K_I = 1,
  K_D = 2
};

class PIDController
{
public:
  explicit PIDController() {
    K_P_ = 1.0;
    K_I_ = 0.0;
    K_D_ = 0.0;
    error_ = 0.0;
    error_integral_ = 0.0;
    error_derivative_ = 0.0;
  }
  explicit PIDController(const std::vector<double>& gains)
  {
    K_P_ = gains.at(K_P);
    K_I_ = gains.at(K_I);
    K_D_ = gains.at(K_D);
    error_ = 0.0;
    error_integral_ = 0.0;
    error_derivative_ = 0.0;
  }
  virtual ~PIDController() {}

private:
  float K_P_;
  float K_I_;
  float K_D_;
  float error_;
  float error_integral_;
  float error_derivative_;
  friend class CarlaAckermannController;

  float run_step(float target_value, float current_value)\
  {
    float prev_error = error_;
    error_ = target_value - current_value;
    error_integral_ = std::clamp(error_integral_+error_, float(-40.0), float(40.0));
    error_derivative_ = error_ - prev_error;
    
    float output = K_P_ * error_ + K_I_ * error_integral_ + K_D_ * error_derivative_;
    // return std::clamp(output, float(0.0), float(1.0)); //steering, throttle range
    return output;
  }
};

#endif  // VEHICLE_PID_CONTROLLER_HPP_

