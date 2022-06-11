/*********************************************************************
 * Copyright (c) 2019, SoftBank Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Softbank Corp. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/

/** NOTE *************************************************************
 * This program had been developed by Michael T. Boulet at MIT under
 * the BSD 3-clause License until Dec. 2016. Since Nov. 2019, Softbank
 * Corp. takes over development as new packages.
 ********************************************************************/

#include "vesc_driver/vesc_driver.h"

#include "rclcpp_components/register_node_macro.hpp"

using std::placeholders::_1;

namespace vesc_driver {
VescDriver::VescDriver(rclcpp::NodeOptions options)
    : Node("vesc_driver", options),
      vesc_(std::bind(&VescDriver::vescErrorCallback, this, std::placeholders::_1)) {
    this->declare_parameter("port", rclcpp::PARAMETER_STRING);

    // TODO handle parameter changes during runtime
    duty_cycle_limit_ = createCommandLimit("duty_cycle", -1.0, 1.0);
    current_limit_ = createCommandLimit("current");
    brake_limit_ = createCommandLimit("brake");
    speed_limit_ = createCommandLimit("speed");
    position_limit_ = createCommandLimit("position");

    // get vesc serial port address
    std::string port;
    if (!this->get_parameter("port", port)) {
        RCLCPP_FATAL(this->get_logger(), "VESC communication port parameter required.");
        rclcpp::shutdown();
        return;
    }

    vesc_.start(port);

    // create vesc state (telemetry) publisher
    state_pub_ = this->create_publisher<vesc_msgs::msg::VescStateStamped>("sensors/core", 10);

    // subscribe to motor and servo command topics
    duty_cycle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "commands/motor/duty_cycle", 10, std::bind(&VescDriver::dutyCycleCallback, this, _1));
    current_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "commands/motor/current", 10, std::bind(&VescDriver::currentCallback, this, _1));
    brake_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "commands/motor/brake", 10, std::bind(&VescDriver::brakeCallback, this, _1));
    speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "commands/motor/speed", 10, std::bind(&VescDriver::speedCallback, this, _1));
    position_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "commands/motor/position", 10, std::bind(&VescDriver::positionCallback, this, _1));
}

VescDriver::~VescDriver() { vesc_.stop(); }

void VescDriver::vescErrorCallback(const std::string& error) { RCLCPP_ERROR(this->get_logger(), "%s", error.c_str()); }

/**
 * @param duty_cycle Commanded VESC duty cycle. Valid range for this driver is -1 to +1. However,
 *                   note that the VESC may impose a more restrictive bounds on the range depending
 *                   on its configuration, e.g. absolute value is between 0.05 and 0.95.
 */
void VescDriver::dutyCycleCallback(const std_msgs::msg::Float64::SharedPtr duty_cycle) {
    vesc_.setDutyCycle(duty_cycle_limit_.clip(duty_cycle->data));
}

/**
 * @param current Commanded VESC current in Amps. Any value is accepted by this driver. However,
 *                note that the VESC may impose a more restrictive bounds on the range depending on
 *                its configuration.
 */
void VescDriver::currentCallback(const std_msgs::msg::Float64::SharedPtr current) {
    vesc_.setCurrent(current_limit_.clip(current->data));
}

/**
 * @param brake Commanded VESC braking current in Amps. Any value is accepted by this driver.
 *              However, note that the VESC may impose a more restrictive bounds on the range
 *              depending on its configuration.
 */
void VescDriver::brakeCallback(const std_msgs::msg::Float64::SharedPtr brake) {
    vesc_.setBrake(brake_limit_.clip(brake->data));
}

/**
 * @param speed Commanded VESC speed in rad/s. Although any value is accepted by this driver, the VESC may impose a
 *              more restrictive bounds on the range depending on its configuration.
 */
void VescDriver::speedCallback(const std_msgs::msg::Float64::SharedPtr speed) {
    vesc_.setSpeed(speed_limit_.clip(speed->data));
}

/**
 * @param position Commanded VESC motor position in radians. Any value is accepted by this driver.
 *                 Note that the VESC must be in encoder mode for this command to have an effect.
 */
void VescDriver::positionCallback(const std_msgs::msg::Float64::SharedPtr position) {
    // ROS uses radians but VESC seems to use degrees. Convert to degrees.
    double position_deg = position_limit_.clip(position->data) * 180.0 / M_PI;
    vesc_.setPosition(position_deg);
}

void VescDriver::waitForStateAndPublish() {
    vesc_.wait_for_status(&vesc_status);

    auto state_msg = vesc_msgs::msg::VescStateStamped();
    state_msg.header.stamp = this->now();
    state_msg.state.connection_state = vesc_status.connection_state;
    state_msg.state.fw_major = vesc_status.fw_version_major;
    state_msg.state.fw_minor = vesc_status.fw_version_minor;
    state_msg.state.voltage_input = vesc_status.voltage_input;
    state_msg.state.temperature_pcb = vesc_status.temperature_pcb;
    state_msg.state.current_motor = vesc_status.current_motor;
    state_msg.state.current_input = vesc_status.current_input;
    state_msg.state.speed = vesc_status.speed_erpm;
    state_msg.state.duty_cycle = vesc_status.duty_cycle;
    state_msg.state.charge_drawn = vesc_status.charge_drawn;
    state_msg.state.charge_regen = vesc_status.charge_regen;
    state_msg.state.energy_drawn = vesc_status.energy_drawn;
    state_msg.state.energy_regen = vesc_status.energy_regen;
    state_msg.state.displacement = vesc_status.displacement;
    state_msg.state.distance_traveled = vesc_status.distance_traveled;
    state_msg.state.fault_code = vesc_status.fault_code;

    state_pub_->publish(state_msg);
}

VescDriver::CommandLimit VescDriver::createCommandLimit(const std::string& name,
                                                        const std::optional<double>& min_lower,
                                                        const std::optional<double>& max_upper) {
    this->declare_parameter(name + "_min", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(name + "_max", rclcpp::PARAMETER_DOUBLE);

    CommandLimit commandLimit;
    commandLimit.name = name;

    // check if user's minimum value is outside of the range min_lower to max_upper
    double param_min;
    if (this->get_parameter(name + "_min", param_min)) {
        if (min_lower && param_min < *min_lower) {
            commandLimit.lower = *min_lower;
            RCLCPP_WARN_STREAM(this->get_logger(),
                               "Parameter " << name << "_min (" << param_min << ") is less than the feasible minimum ("
                                            << *min_lower << ").");
        } else if (max_upper && param_min > *max_upper) {
            commandLimit.lower = *max_upper;
            RCLCPP_WARN_STREAM(this->get_logger(),
                               "Parameter " << name << "_min (" << param_min
                                            << ") is greater than the feasible maximum (" << *max_upper << ").");
        } else {
            commandLimit.lower = param_min;
        }
    } else if (min_lower) {
        commandLimit.lower = *min_lower;
    }

    // check if the uers' maximum value is outside of the range min_lower to max_upper
    double param_max;
    if (this->get_parameter(name + "_max", param_max)) {
        if (min_lower && param_max < *min_lower) {
            commandLimit.upper = *min_lower;
            RCLCPP_WARN_STREAM(this->get_logger(),
                               "Parameter " << name << "_max (" << param_max << ") is less than the feasible minimum ("
                                            << *min_lower << ").");
        } else if (max_upper && param_max > *max_upper) {
            commandLimit.upper = *max_upper;
            RCLCPP_WARN_STREAM(this->get_logger(),
                               "Parameter " << name << "_max (" << param_max
                                            << ") is greater than the feasible maximum (" << *max_upper << ").");
        } else {
            commandLimit.upper = param_max;
        }
    } else if (max_upper) {
        commandLimit.upper = *max_upper;
    }

    // check for min > max
    if (commandLimit.upper && commandLimit.lower && *commandLimit.lower > *commandLimit.upper) {
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "Parameter " << name << "_max (" << *commandLimit.upper << ") is less than parameter "
                                        << name << "_min (" << *commandLimit.lower << ").");
        double temp(*commandLimit.lower);
        commandLimit.lower = *commandLimit.upper;
        commandLimit.upper = temp;
    }

    std::ostringstream oss;
    oss << "  " << name << " limit: ";
    if (commandLimit.lower)
        oss << *commandLimit.lower << " ";
    else
        oss << "(none) ";
    if (commandLimit.upper)
        oss << *commandLimit.upper;
    else
        oss << "(none)";
    RCLCPP_DEBUG_STREAM(this->get_logger(), oss.str());

    return commandLimit;
}

double VescDriver::CommandLimit::clip(double value) {
    // auto& clk = *node->get_clock();
    if (lower && value < lower) {
        // RCLCPP_INFO_THROTTLE(node->get_logger(),
        //                      clk,
        //                      10000,
        //                      "%s command value (%f) below minimum limit (%f), clipping.",
        //                      name.c_str(),
        //                      value,
        //                      *lower);
        return *lower;
    }
    if (upper && value > upper) {
        // RCLCPP_INFO_THROTTLE(node->get_logger(),
        //                      clk,
        //                      10000,
        //                      "%s command value (%f) above maximum limit (%f), clipping.",
        //                      name.c_str(),
        //                      value,
        //                      *upper);
        return *upper;
    }
    return value;
}

} // namespace vesc_driver

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_driver::VescDriver)
