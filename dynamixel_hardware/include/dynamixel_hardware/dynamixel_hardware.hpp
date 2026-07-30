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
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "dynamixel_hardware/visiblity_control.h"
#include "rclcpp/macros.hpp"

namespace dynamixel_hardware
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using return_type = hardware_interface::return_type;

struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct Joint
{
  JointValue state{};
  JointValue command{};
  JointValue prev_command{};
  int control_mode{0};
  double gear_ratio{1.0};

  // Last raw integer values sent to the servo — skip SyncWrite when unchanged
  int32_t prev_pos_raw{std::numeric_limits<int32_t>::min()};
  int32_t prev_vel_raw{std::numeric_limits<int32_t>::min()};
  int16_t prev_curt_raw{std::numeric_limits<int16_t>::min()};

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
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & info) override;

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
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  return_type enable_torque(const bool enabled, const bool force = false);

  return_type set_control_mode();

  return_type reset_command();

  CallbackReturn set_joint_positions();
  CallbackReturn set_joint_velocities();
  CallbackReturn set_joint_currents();
  CallbackReturn set_joint_params();

  // Workbench is used only during init/configure for ping, getItemInfo,
  // setXxxControlMode, itemWrite, torqueOn/Off, and unit conversion.
  // All hot-path read/write uses the raw SDK objects below.
  DynamixelWorkbench dynamixel_workbench_;
  std::map<const char * const, const ControlItem *> control_items_;

  std::vector<Joint> joints_;
  std::vector<uint8_t> joint_ids_;
  std::unordered_map<uint8_t, int> joint_id_to_index_;  // servo ID → joints_ index, O(1) lookup
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

  // Single shared port — obtained via PortHandler::getPortHandler() after workbench init,
  // so workbench and all SDK objects below use the same file descriptor.
  // Raw pointer: lifetime is managed by the SDK's internal singleton registry.
  dynamixel::PortHandler *   port_handler_{nullptr};
  dynamixel::PacketHandler * packet_handler_{nullptr};

  // Fast Sync Read (0x8A) — one per physical bus (TTL / RS-485)
  std::unique_ptr<dynamixel::GroupFastSyncRead> fast_sync_read_ttl_;
  std::unique_ptr<dynamixel::GroupFastSyncRead> fast_sync_read_rs_;

  // Sync Write — replaces workbench syncWrite() on the hot path
  std::unique_ptr<dynamixel::GroupSyncWrite> sync_write_position_;
  std::unique_ptr<dynamixel::GroupSyncWrite> sync_write_velocity_;
  std::unique_ptr<dynamixel::GroupSyncWrite> sync_write_current_;
};
}  // namespace dynamixel_hardware

#endif  // DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_
