/*********************************************************************
 * Copyright (c) 2019, SoftBank Corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************/

#include "vesc_hw_interface/vesc_servo_controller.h"

namespace vesc_hw_interface
{
void VescServoController::init(ros::NodeHandle nh, VescInterface* interface_ptr)
{
  // initializes members
  if (interface_ptr == NULL)
  {
    ros::shutdown();
  }
  else
  {
    interface_ptr_ = interface_ptr;
  }

  calibration_flag_ = true;
  zero_position_ = 0.0;
  error_integ_ = 0.0;

  // reads parameters
  nh.param("servo/Kp", Kp_, 50.0);
  nh.param("servo/Ki", Ki_, 0.0);
  nh.param("servo/Kd", Kd_, 1.0);
  nh.param("servo/calibration_current", calibration_current_, 6.0);
  nh.param("servo/calibration_duty", calibration_duty_, 0.1);
  nh.param<std::string>("servo/calibration_mode", calibration_mode_, "current");
  nh.param("servo/calibration_position", calibration_position_, 0.0);

  // shows parameters
  ROS_INFO("[Servo Gains] P: %f, I: %f, D: %f", Kp_, Ki_, Kd_);
  if (calibration_mode_ == CURRENT)
  {
    ROS_INFO("[Servo Calibration] Mode: %s, value: %f", CURRENT.data(), calibration_current_);
  }
  else if (calibration_mode_ == DUTY)
  {
    ROS_INFO("[Servo Calibration] Mode: %s, value: %f", DUTY.data(), calibration_duty_);
  }
  else
  {
    ROS_ERROR("[Servo Calibration] Invalid mode");
  }

  return;
}

void VescServoController::control(const double position_reference, const double position_current)
{
  // executes caribration
  if (calibration_flag_)
  {
    calibrate(position_current);

    // initializes/resets control variables
    time_previous_ = ros::Time::now();
    position_previous_ = position_current;
    position_reference_previous_ = position_current;
    error_previous_ = 0.0;
    return;
  }

  const ros::Time time_current = ros::Time::now();
  const double dt = (time_current - time_previous_).toSec();

  // calculates PD control
  const double error_current = position_reference - position_current;
  const double u_p = Kp_ * error_current + Kd_;
  const double u_pd = u_p + Kd_ * (smoothDifferentiate(position_current, position_previous_, dt) + (position_reference - position_reference_previous_) / dt);
  // const double u_pd = Kp_ * error_current + Kd_ * (error_current - error_previous_) / dt;

  double u = 0.0;

  // calculates I control if PD input is not saturated
  if (isSaturated(u_pd))
  {
    u = saturate(u_pd);
  }
  else
  {
    double error_integ_new = error_integ_ + (error_current + error_previous_) / 2.0 * dt;
    const double u_pid = u_pd + Ki_ * error_integ_new;

    // not use I control if PID input is saturated
    // since error integration causes bugs
    if (isSaturated(u_pid))
    {
      u = u_pd;
    }
    else
    {
      u = u_pid;
      error_integ_ = error_integ_new;
    }
  }

  // updates previous data
  error_previous_ = error_current;
  time_previous_ = time_current;
  position_previous_ = position_current;
  position_reference_previous_ = position_reference;

  // command duty
  interface_ptr_->setDutyCycle(u);

  return;
}

double VescServoController::getZeroPosition() const
{
  return zero_position_;
}

void VescServoController::executeCalibration()
{
  calibration_flag_ = true;

  return;
}

bool VescServoController::calibrate(const double position_current)
{
  static double position_previous;
  static uint16_t step;

  // sends a command for calibration
  if (calibration_mode_ == CURRENT)
  {
    interface_ptr_->setCurrent(calibration_current_);
  }
  else if (calibration_mode_ == DUTY)
  {
    interface_ptr_->setDutyCycle(calibration_duty_);
  }
  else
  {
    ROS_ERROR("Please set the calibration mode surely");
    return false;
  }

  step++;

  if (step % 20 == 0 && position_current == position_previous)
  {
    // finishes calibrating
    interface_ptr_->setCurrent(0.0);
    calibration_flag_ = false;
    step = 0;
    zero_position_ = position_current - calibration_position_;

    ROS_INFO("Calibration Finished");
    return true;
  }
  else
  {
    // continues calibration
    position_previous = position_current;

    return false;
  }
}

bool VescServoController::isSaturated(const double arg) const
{
  if (std::abs(arg) > 1.0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

double VescServoController::saturate(const double arg) const
{
  if (arg > 1.0)
  {
    return 1.0;
  }
  else if (arg < -1.0)
  {
    return -1.0;
  }
  else
  {
    return arg;
  }
}

double VescServoController::smoothDifferentiate(const double position_current, const double position_previous, const double dt) const
{
  static double velocity_out = 0.0;
  const double smoothing_timeout = 1.0;
  const double smoothing_timeout_lpf = 0.99; // TODO: Calculate time constant
  static double same_position_time = 0.0;
  if(position_current == position_previous)
  {
    if(same_position_time < smoothing_timeout)
    {
      same_position_time += dt; // Output is same as previous value
    }
    else
    {
      velocity_out *= smoothing_timeout_lpf;
    }
  }
  else
  {
    velocity_out = (position_current - position_previous) / (dt + same_position_time);
    same_position_time = 0.0;
    if(same_position_time > smoothing_timeout)
    {
      velocity_out *= smoothing_timeout_lpf;
    }
  }
  return velocity_out;
}
}  // namespace vesc_hw_interface
