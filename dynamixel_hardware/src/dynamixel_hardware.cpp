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

#include "dynamixel_hardware/dynamixel_hardware.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dynamixel_hardware
{
constexpr const char * kDynamixelHardware = "DynamixelHardware";
constexpr uint8_t kGoalPositionIndex = 0;
constexpr uint8_t kGoalVelocityIndex = 1;
constexpr uint8_t kGoalCurrentIndex = 2;
constexpr uint8_t kPresentPositionVelocityCurrentIndex = 0;
constexpr const char * kGoalPositionItem = "Goal_Position";
constexpr const char * kGoalVelocityItem = "Goal_Velocity";
constexpr const char * kGoalCurrentItem = "Goal_Current";
constexpr const char * kMovingSpeedItem = "Moving_Speed";
constexpr const char * kPresentPositionItem = "Present_Position";
constexpr const char * kPresentVelocityItem = "Present_Velocity";
constexpr const char * kPresentSpeedItem = "Present_Speed";
constexpr const char * kPresentCurrentItem = "Present_Current";
constexpr const char * kPresentLoadItem = "Present_Load";
constexpr const char * const kExtraJointParameters[] = {
  "Profile_Velocity",
  "Profile_Acceleration",
  "Position_P_Gain",
  "Position_I_Gain",
  "Position_D_Gain",
  "Velocity_P_Gain",
  "Velocity_I_Gain",
  "Feedforward_2nd_Gain",
  "Feedforward_1st_Gain",
  "Goal_Current",
};

CallbackReturn DynamixelHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "init");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  joints_.resize(info_.joints.size(), Joint());
  joint_ids_.resize(info_.joints.size(), 0);

  for (uint i = 0; i < info_.joints.size(); i++) {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
    if (info_.joints[i].parameters.find("control_mode") != info_.joints[i].parameters.end()) {
      joints_[i].control_mode = std::stoi(info_.joints[i].parameters.at("control_mode"));
      if (joints_[i].control_mode == 0 ||
          joints_[i].control_mode == 4 ||
          joints_[i].control_mode == 5 ||
          joints_[i].control_mode == 6)
      {
        joint_pos_ids_.push_back(i);
        joint_pos_real_ids_.push_back(joint_ids_[i]);
      } else if (joints_[i].control_mode == 1 ||
                 joints_[i].control_mode == 7)
      {
        joint_vel_ids_.push_back(i);
        joint_vel_real_ids_.push_back(joint_ids_[i]);
      } else if (joints_[i].control_mode == 2 ||
                 joints_[i].control_mode == 3)
      {
        joint_curt_ids_.push_back(i);
        joint_curt_real_ids_.push_back(joint_ids_[i]);
      } else {
        RCLCPP_ERROR(
          rclcpp::get_logger(kDynamixelHardware), "Control mode not implemented: %d", joints_[i].control_mode);
        return CallbackReturn::ERROR;
      }
    }
    if (info_.joints[i].parameters.find("gear_ratio") != info_.joints[i].parameters.end()) {
      joints_[i].gear_ratio = std::stod(info_.joints[i].parameters.at("gear_ratio"));
    }
    if (
      info_.joints[i].parameters.at("interface") == "TTL" ||
      info_.joints[i].parameters.at("interface") == "ttl")
    {
      joint_ids_ttl_.push_back(joint_ids_[i]);
    }else if (
      info_.joints[i].parameters.at("interface") == "RS" ||
      info_.joints[i].parameters.at("interface") == "rs")
    {
      joint_ids_rs_.push_back(joint_ids_[i]);
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "joint_id %d: %d", i, joint_ids_[i]);
  }

  if (
    info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end() &&
    (info_.hardware_parameters.at("use_dummy") == "true" ||
    info_.hardware_parameters.at("use_dummy") == "True"))
  {
    use_dummy_ = true;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "dummy mode");
    return CallbackReturn::SUCCESS;
  }

  // TODO: multiple port support
  auto port_name = info_.hardware_parameters.at("port_name");
  auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
  const char * log = nullptr;

  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "port_name: %s", port_name.c_str());
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "baud_rate: %d", baud_rate);

  if (!dynamixel_workbench_.init(port_name.c_str(), baud_rate, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  for (uint i = 0; i < info_.joints.size(); ++i) {
    uint16_t model_number = 0;
    if (!dynamixel_workbench_.ping(joint_ids_[i], &model_number, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      return CallbackReturn::ERROR;
    }
  }

  const ControlItem * goal_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalPositionItem);
  if (goal_position == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * goal_velocity =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalVelocityItem);
  if (goal_velocity == nullptr) {
    goal_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kMovingSpeedItem);
  }
  if (goal_velocity == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * goal_current =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalCurrentItem);
  if (goal_current == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * present_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentPositionItem);
  if (present_position == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * present_velocity =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentVelocityItem);
  if (present_velocity == nullptr) {
    present_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentSpeedItem);
  }
  if (present_velocity == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * present_current =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentCurrentItem);
  if (present_current == nullptr) {
    present_current = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentLoadItem);
  }
  if (present_current == nullptr) {
    return CallbackReturn::ERROR;
  }

  control_items_[kGoalPositionItem] = goal_position;
  control_items_[kGoalVelocityItem] = goal_velocity;
  control_items_[kGoalCurrentItem] = goal_current;
  control_items_[kPresentPositionItem] = present_position;
  control_items_[kPresentVelocityItem] = present_velocity;
  control_items_[kPresentCurrentItem] = present_current;

  if (!dynamixel_workbench_.addSyncWriteHandler(
      control_items_[kGoalPositionItem]->address, control_items_[kGoalPositionItem]->data_length,
      &log))
  {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  if (!dynamixel_workbench_.addSyncWriteHandler(
      control_items_[kGoalVelocityItem]->address, control_items_[kGoalVelocityItem]->data_length,
      &log))
  {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  if (!dynamixel_workbench_.addSyncWriteHandler(
      control_items_[kGoalCurrentItem]->address, control_items_[kGoalCurrentItem]->data_length,
      &log))
  {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  uint16_t start_address = std::min(
    control_items_[kPresentPositionItem]->address, control_items_[kPresentCurrentItem]->address);
  uint16_t read_length = control_items_[kPresentPositionItem]->data_length +
    control_items_[kPresentVelocityItem]->data_length +
    control_items_[kPresentCurrentItem]->data_length + 2;
  if (!dynamixel_workbench_.addSyncReadHandler(start_address, read_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::on_configure(const rclcpp_lifecycle::State & /* previous_state */) 
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "configure");

  for (uint i = 0; i < joints_.size(); i++) {
    if (use_dummy_ && std::isnan(joints_[i].state.position)) {
      joints_[i].state.position = 0.0;
      joints_[i].state.velocity = 0.0;
      joints_[i].state.effort = 0.0;
    }
  }

  if (read(rclcpp::Time{}, rclcpp::Duration(0, 0)) == return_type::ERROR) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Read failed in on_configure");
    return CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.find("gripper_open_direction") != info_.hardware_parameters.end()) {
    gripper_open_direction_ = info_.hardware_parameters["gripper_open_direction"];
  }


  enable_torque(false);
  set_control_mode();
  set_joint_params();
  // Ideally torque should be enabled in on_activate(), but this appears to cause issues 
  // due to conflict with RT loop read/write calls, so it is here instead
  enable_torque(true);

  return CallbackReturn::SUCCESS;
}

// TODO - add on_cleanup to disable torque

std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    if (joints_[i].control_mode == 0 ||
        joints_[i].control_mode == 4 ||
        joints_[i].control_mode == 5 ||
        joints_[i].control_mode == 6) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    } else if (joints_[i].control_mode == 1 ||
               joints_[i].control_mode == 7) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
    } else if (joints_[i].control_mode == 2 ||
               joints_[i].control_mode == 3) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].command.effort));
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger(kDynamixelHardware), "Control mode not implemented");
      return {};
    }
  }

  return command_interfaces;
}

CallbackReturn DynamixelHardware::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "activate");
  reset_command();

  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "deactivate");
  return CallbackReturn::SUCCESS;
}

return_type DynamixelHardware::read(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    return return_type::OK;
  }

  std::vector<uint8_t>* ids_each[] = {nullptr, nullptr};
  if(!joint_ids_ttl_.empty()){
    ids_each[0] = &joint_ids_ttl_;
  }
  if(!joint_ids_rs_.empty()){
    ids_each[1] = &joint_ids_rs_;
  }

  for(auto& idt: ids_each){
    if(idt == nullptr){
      continue;
    }

    std::vector<uint8_t> ids(idt->size(), 0);
    std::vector<int32_t> positions(idt->size(), 0);
    std::vector<int32_t> velocities(idt->size(), 0);
    std::vector<int32_t> currents(idt->size(), 0);
    std::copy(idt->begin(), idt->end(), ids.begin());
    const char * log = nullptr;

    if (!dynamixel_workbench_.syncRead(
          kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(), &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    }
    if (!dynamixel_workbench_.getSyncReadData(
          kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
          control_items_[kPresentCurrentItem]->address,
          control_items_[kPresentCurrentItem]->data_length, currents.data(), &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    }
    if (!dynamixel_workbench_.getSyncReadData(
          kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
          control_items_[kPresentVelocityItem]->address,
          control_items_[kPresentVelocityItem]->data_length, velocities.data(), &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    }
    if (!dynamixel_workbench_.getSyncReadData(
          kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
          control_items_[kPresentPositionItem]->address,
          control_items_[kPresentPositionItem]->data_length, positions.data(), &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    }

    for(uint i = 0; i < ids.size(); i++){
      auto it = std::find(joint_ids_.begin(), joint_ids_.end(), ids[i]);
      if(it != joint_ids_.end()){
        int index = std::distance(joint_ids_.begin(), it);
        joints_[index].state.position = dynamixel_workbench_.convertValue2Radian(
          ids[i], positions[i]) / joints_[i].gear_ratio;
        joints_[index].state.velocity = dynamixel_workbench_.convertValue2Velocity(
          ids[i], velocities[i]) / joints_[i].gear_ratio;
        joints_[index].state.effort = dynamixel_workbench_.convertValue2Current(
          currents[i]) * joints_[i].gear_ratio;
      }
    }
  }

  return return_type::OK;
}

return_type DynamixelHardware::write(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    for (auto & joint : joints_) {
      joint.state.position = joint.command.position;
      joint.state.velocity = joint.command.velocity;
      joint.state.effort = joint.command.effort;
      joint.prev_command.position = joint.command.position;
      joint.prev_command.velocity = joint.command.velocity;
      joint.prev_command.effort = joint.command.effort;
    }
    return return_type::OK;
  }

    for (size_t i = 0; i < joints_.size(); ++i) {
      if (joints_[i].control_mode == 6) {
          // Only store commands that came from the action server (not our frozen values)
          if (!gripper_locked_) {
            last_user_cmd_pos_ = joints_[i].command.position;

            // RCLCPP_INFO(
            //     rclcpp::get_logger(kDynamixelHardware),
            //     "[DEBUG][USER_CMD] Saved user command pos=%.3f",
            //     last_user_cmd_pos_);
        }

      }
  }
  
    //----------------------------------------------
    //  GRIPPER SAFETY LOGIC (FINAL CLEAN VERSION)
    //----------------------------------------------

    const double kCurrentLimit_mA   = 1200.0;  // Overcurrent threshold
    const double kReleaseCurrent_mA = 400.0;   // Current indicating load is gone
    const double kOpenThresholdRad  = 0.15;    // Required motion to count as "opening"
    const double kStillEpsilonRad   = 0.02;    // Position delta to consider "not moving"

    //-------------------------------
    // A) Detect overcurrent → LOCK
    //-------------------------------
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (joints_[i].control_mode == 6 &&
          !std::isnan(joints_[i].state.effort) &&
          joints_[i].state.effort > kCurrentLimit_mA)
      {
        if (!gripper_locked_) {
          gripper_locked_       = true;
          gripper_hold_position_ = joints_[i].state.position;

          RCLCPP_WARN(
            rclcpp::get_logger(kDynamixelHardware),
            "[LOCK] Overcurrent on %s. Holding at %.3f rad",
            info_.joints[i].name.c_str(),
            gripper_hold_position_);
        }
      }
    }

    //-------------------------------
    // B) Detect USER OPEN INTENT
    //-------------------------------
    bool user_requested_open = false;

    if (gripper_locked_) {
        for (size_t i = 0; i < joints_.size(); ++i) {
            if (joints_[i].control_mode == 6) {

                double user_cmd = last_user_cmd_pos_;

                if (std::isnan(user_cmd)) continue;

                bool open_intent =
                    (gripper_open_direction_ == "positive")
                        ? (user_cmd > gripper_hold_position_ + kOpenThresholdRad)
                        : (user_cmd < gripper_hold_position_ - kOpenThresholdRad);
                
                RCLCPP_INFO(
                  rclcpp::get_logger(kDynamixelHardware),
                  "[DEBUG][OPEN_CHECK] hold=%.3f user_cmd=%.3f threshold=%.3f → open_intent=%d",
                  gripper_hold_position_,
                  user_cmd,
                  kOpenThresholdRad,
                  open_intent);


                if (open_intent) {
                    user_requested_open = true;
                }
            }
        }
    }


    //----------------------------------------------
    // C) Detect SAFE passive unlock (object removed)
    //----------------------------------------------
    bool passive_safe_unlock = false;

    if (gripper_locked_) {
      for (size_t i = 0; i < joints_.size(); ++i) {
        if (joints_[i].control_mode == 6) {

          double current = joints_[i].state.effort;
          double cmd     = joints_[i].command.position;
          double pos     = joints_[i].state.position;

          bool low_current = (!std::isnan(current) && current < kReleaseCurrent_mA);
          bool not_moving  = (std::fabs(cmd - pos) < kStillEpsilonRad);

          // RCLCPP_INFO(
          //   rclcpp::get_logger(kDynamixelHardware),
          //   "[DEBUG][PASSIVE] current=%.1f cmd=%.3f pos=%.3f diff=%.3f → low_current=%d not_moving=%d",
          //   current,
          //   cmd,
          //   pos,
          //   std::fabs(cmd - pos),
          //   low_current,
          //   not_moving);


          if (low_current && not_moving && !user_requested_open) {
            passive_safe_unlock = true;
          }
        }
      }
    }

    //---------------------------
    // D) UNLOCK DECISION
    //---------------------------

    // 1) User wants to open → ALWAYS UNLOCK (ignores current)
    if (gripper_locked_ && user_requested_open) {
      gripper_locked_ = false;

      RCLCPP_WARN(
        rclcpp::get_logger(kDynamixelHardware),
        "[UNLOCK] User requested OPEN → unlocking immediately");
    }

    // 2) Passive safe unlock (object removed, relaxed)
    else if (gripper_locked_ && passive_safe_unlock) {
      gripper_locked_ = false;

      RCLCPP_WARN(
        rclcpp::get_logger(kDynamixelHardware),
        "[UNLOCK] Current safe & no motion → passive unlock");
    }

    //------------------------------------------
    // E) If still locked → FREEZE MOVEMENT
    //------------------------------------------
    if (gripper_locked_) {

      for (size_t i = 0; i < joints_.size(); ++i) {
        if (joints_[i].control_mode == 6) {
          joints_[i].command.position      = gripper_hold_position_;
          joints_[i].prev_command.position = gripper_hold_position_;
        }
      }

      // Keep gripper still
      set_joint_positions();
      return return_type::OK;
    }


  // Velocity control
  if (std::any_of(
      joints_.cbegin(), joints_.cend(), [](auto j) {
        return j.command.velocity != j.prev_command.velocity;
      }))
  {
    set_joint_velocities();
    return return_type::OK;
  }

  // Position control
  if (std::any_of(
      joints_.cbegin(), joints_.cend(), [](auto j) {
        return j.command.position != j.prev_command.position;
      }))
  {
    set_joint_positions();
    return return_type::OK;
  }

  // Effort control
  if (std::any_of(
      joints_.cbegin(), joints_.cend(), [](auto j) {
        return j.command.effort != j.prev_command.effort;
      })) 
  {
    set_joint_currents();
    return return_type::OK;
  }
  // 🔍 DEBUG: ALWAYS print hand_joint state (control_mode == 6)
  for (size_t i = 0; i < joints_.size(); i++) {
    if (joints_[i].control_mode == 6) {
      RCLCPP_INFO(
        rclcpp::get_logger("DynamixelHardware"),
        "[DEBUG] hand_joint pos=%.3f state_effort=%.3f cmd_effort=%.3f",
        joints_[i].state.position,
        joints_[i].state.effort,   // measured current
        joints_[i].command.effort  
      );
    }
  }
  
  return return_type::OK;
  
}

return_type DynamixelHardware::enable_torque(const bool enabled)
{
  const char * log = nullptr;

  if (enabled && !torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      if (!dynamixel_workbench_.torqueOn(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    // reset_command();
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque enabled");
  } else if (!enabled && torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      if (!dynamixel_workbench_.torqueOff(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque disabled");
  }

  torque_enabled_ = enabled;
  return return_type::OK;
}

return_type DynamixelHardware::set_control_mode()
{
  const char * log = nullptr;

  bool torque_enabled = torque_enabled_;
  if (torque_enabled) {
    enable_torque(false);
  }

  for (uint i = 0; i < joint_ids_.size(); ++i) {
    if (joints_[i].control_mode == 0) {
      if (!dynamixel_workbench_.setPositionControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    } else if (joints_[i].control_mode == 1) {
      if (!dynamixel_workbench_.setVelocityControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    } else if (joints_[i].control_mode == 2) {
      if (!dynamixel_workbench_.setTorqueControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    } else if (joints_[i].control_mode == 3) {
      if (!dynamixel_workbench_.setCurrentControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    } else if (joints_[i].control_mode == 4) {
      if (!dynamixel_workbench_.setExtendedPositionControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    } else if (joints_[i].control_mode == 5) {
      if (!dynamixel_workbench_.setMultiTurnControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    } else if (joints_[i].control_mode == 6) {
      if (!dynamixel_workbench_.setCurrentBasedPositionControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    } else if (joints_[i].control_mode == 7) {
      if (!dynamixel_workbench_.setPWMControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger(kDynamixelHardware), "Control mode not implemented");
      return return_type::ERROR;
    }
  }
  if (torque_enabled) {
    enable_torque(true);
  }

  return return_type::OK;
}

return_type DynamixelHardware::reset_command()
{
  for (uint i = 0; i < joints_.size(); i++) {
    joints_[i].command.position = joints_[i].state.position;
    joints_[i].command.velocity = 0.0;
    joints_[i].command.effort = 0.0;
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
  }

  return return_type::OK;
}

CallbackReturn DynamixelHardware::set_joint_positions()
{
  const char * log = nullptr;
  std::vector<int32_t> commands(joint_pos_ids_.size(), 0);
  std::vector<uint8_t> ids(joint_pos_ids_.size(), 0);

  std::copy(joint_pos_ids_.begin(), joint_pos_ids_.end(), ids.begin());
  for (uint i = 0; i < ids.size(); i++) {
    joints_[ids[i]].prev_command.position = joints_[ids[i]].command.position;
    commands[i] = dynamixel_workbench_.convertRadian2Value(
      joint_ids_[ids[i]], static_cast<float>(joints_[ids[i]].command.position * joints_[ids[i]].gear_ratio));
  }
  if (!dynamixel_workbench_.syncWrite(
      kGoalPositionIndex, joint_pos_real_ids_.data(), ids.size(), commands.data(), 1, &log))
  {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::set_joint_velocities()
{
  const char * log = nullptr;
  std::vector<int32_t> commands(joint_vel_ids_.size(), 0);
  std::vector<uint8_t> ids(joint_vel_ids_.size(), 0);

  std::copy(joint_vel_ids_.begin(), joint_vel_ids_.end(), ids.begin());
  for (uint i = 0; i < ids.size(); i++) {
    joints_[ids[i]].prev_command.velocity = joints_[ids[i]].command.velocity;
    commands[i] = dynamixel_workbench_.convertVelocity2Value(
      joint_ids_[ids[i]], static_cast<float>(joints_[ids[i]].command.velocity * joints_[ids[i]].gear_ratio));
  }
  if (!dynamixel_workbench_.syncWrite(
      kGoalVelocityIndex, joint_vel_real_ids_.data(), ids.size(), commands.data(), 1, &log))
  {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::set_joint_currents()
{
  const char * log = nullptr;
  std::vector<int32_t> commands(joint_curt_ids_.size(), 0);
  std::vector<uint8_t> ids(joint_curt_ids_.size(), 0);

  std::copy(joint_curt_ids_.begin(), joint_curt_ids_.end(), ids.begin());
  for (uint i = 0; i < ids.size(); i++) {
    joints_[ids[i]].prev_command.effort = joints_[ids[i]].command.effort;
    commands[i] = dynamixel_workbench_.convertCurrent2Value(
      joint_ids_[ids[i]], static_cast<float>(joints_[ids[i]].command.effort * joints_[ids[i]].gear_ratio));
  }
  if (!dynamixel_workbench_.syncWrite(
    kGoalCurrentIndex, joint_curt_real_ids_.data(), ids.size(), commands.data(), 1, &log))
  {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::set_joint_params()
{
  const char * log = nullptr;
  for (uint i = 0; i < info_.joints.size(); ++i) {
    for (auto paramName : kExtraJointParameters) {
      if (info_.joints[i].parameters.find(paramName) != info_.joints[i].parameters.end()) {
        auto value = std::stoi(info_.joints[i].parameters.at(paramName));
        if (!dynamixel_workbench_.itemWrite(joint_ids_[i], paramName, value, &log)) {
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(
          rclcpp::get_logger(
            kDynamixelHardware), "%s set to %d for joint %d", paramName, value, i);
      }
    }
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace dynamixel_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)
