// Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
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

#ifndef DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_
#define DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#include <map>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "dynamixel_hardware/visiblity_control.h"
#include "rclcpp/macros.hpp"

namespace dynamixel_hardware
{
using CallbackReturn = hardware_interface::CallbackReturn;
using return_type = hardware_interface::return_type;

struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct Joint {
  std::string name;
  JointValue state{};
  JointValue command{};
  JointValue prev_command{};
  int control_mode{0};
  double gear_ratio{1.0};

  // Mimic joint parameters
  int mimic_index{-1};  // -1 if not a mimic joint, otherwise index of the source joint
  double mimic_multiplier{1.0};
  double mimic_offset{0.0};
};

enum class ControlMode
{
  Position,
  Velocity,
  Torque,
  Current,
  ExtendedPosition,
  MultiTurn,
  CurrentBasedPosition,
  PWM,
  None,
};

class DynamixelHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DynamixelHardware)

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DYNAMIXEL_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  return_type enable_torque(const bool enabled);

  return_type set_control_mode();

  return_type reset_command();

  CallbackReturn set_joint_positions();
  CallbackReturn set_joint_velocities();
  CallbackReturn set_joint_currents();
  CallbackReturn set_joint_params();

  DynamixelWorkbench dynamixel_workbench_;
  std::map<const char * const, const ControlItem *> control_items_;
  std::vector<Joint> joints_;
  std::vector<uint8_t> joint_ids_;
  std::vector<uint8_t> joint_ids_ttl_;
  std::vector<uint8_t> joint_ids_rs_;
  std::vector<uint8_t> joint_pos_ids_;
  std::vector<uint8_t> joint_vel_ids_;
  std::vector<uint8_t> joint_curt_ids_;
  std::vector<uint8_t> joint_pos_real_ids_;
  std::vector<uint8_t> joint_vel_real_ids_;
  std::vector<uint8_t> joint_curt_real_ids_;

  bool torque_enabled_{false};
  bool use_dummy_{false};
};
}  // namespace dynamixel_hardware

#endif  // DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_
