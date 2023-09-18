// Copyright 2023 Andrea Ostuni - PIC4SeR
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HUSKY_BASE__HUSKY_HARDWARE_HPP_
#define HUSKY_BASE__HUSKY_HARDWARE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"
#include "husky_base/horizon_legacy_wrapper.h"
#include "husky_base/husky_diagnostics.hpp"
#include "husky_base/husky_status.hpp"
#include "husky_msgs/msg/husky_status.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace husky_base
{

class HuskyHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HuskyHardware)

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  void resetTravelOffset();
  double linearToAngular(const double& travel) const;
  double angularToLinear(const double& angle) const;
  void writeCommandsToHardware();
  void limitDifferentialSpeed(double& diff_speed_left, double& diff_speed_right);
  void updateJointsFromHardware();
  void readStatusFromHardware();
  uint8_t isLeft(const std::string& str);

  // ROS Parameters
  std::string serial_port_;
  double polling_timeout_;
  double wheel_diameter_, max_accel_, max_speed_;

  // Store the command for the robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_position_, hw_states_position_offset_, hw_states_velocity_;

  uint8_t left_cmd_joint_index_, right_cmd_joint_index_;
  double control_loop_frequency_;

  std::shared_ptr<husky_status::HuskyStatus> status_node_;
  husky_msgs::msg::HuskyStatus status_msg_;

  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  std::unique_ptr<husky_base::HuskyHardwareSystemDiagnosticTask> system_status_task_;
  std::unique_ptr<husky_base::HuskyHardwarePowerDiagnosticTask> power_status_task_;
  std::unique_ptr<husky_base::HuskyHardwareSafetyDiagnosticTask> safety_status_task_;
  std::unique_ptr<husky_base::HuskySoftwareDiagnosticTask> software_status_task_;
};

}  // namespace husky_base

#endif  // HUSKY_BASE__HUSKY_HARDWARE_HPP_
