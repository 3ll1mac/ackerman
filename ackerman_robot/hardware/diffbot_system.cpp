// Copyright 2021 ros2_control Development Team
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

#include "ackerman_robot/diffbot_system.hpp"

#include <iostream>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ackerman_robot
{
hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "PID values not supplied, using defaults.");
  }

  cfg_.front_left_wheel_name = info_.hardware_parameters["front_left_wheel_name"];
  cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
  cfg_.rear_left_wheel_name= info_.hardware_parameters["rear_left_wheel_name"];
  cfg_.rear_right_wheel_name = info_.hardware_parameters["rear_right_wheel_name"];
 
  

  front_wheel_l_.setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev);
  front_wheel_r_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);
  rear_wheel_l_.setup(cfg_.rear_left_wheel_name, cfg_.enc_counts_per_rev);
  rear_wheel_r_.setup(cfg_.rear_right_wheel_name, cfg_.enc_counts_per_rev);



  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    bool joint_is_steering = joint.name.find("steering") != std::string::npos;

    // Steering joints have a position command interface and a position state interface
    if (joint_is_steering)
    {
      RCLCPP_INFO(get_logger(), "Joint '%s' is a steering joint.", joint.name.c_str());

      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Joint '%s' is a drive joint.", joint.name.c_str());

      // Drive joints have a velocity command interface and a velocity state interface
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }    
  }
 /* Bicyle
  // code
  hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // // END: This part here is for exemplary purposes - Please do not copy to your production code

  hw_interfaces_["steering"] = Joint("left_wheel_steering_joint");

  hw_interfaces_["traction"] = Joint("rear_left_wheel_joint");

 */
 
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
{
  
  /*std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.second.joint_name, hardware_interface::HW_IF_POSITION, &joint.second.state.position));

      RCLCPP_INFO(get_logger(), "JOINT %s", joint.first.c_str());

    if (joint.first == "traction")
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.second.joint_name, hardware_interface::HW_IF_VELOCITY, &joint.second.state.velocity));
    }
  }

  RCLCPP_INFO(get_logger(), "Exported %zu state interfaces.", state_interfaces.size());

  for (auto s : state_interfaces)
  {
    RCLCPP_INFO(get_logger(), "Exported state interface '%s'.", s.get_name().c_str());
  }

  return state_interfaces;*/

  
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    front_wheel_l_.name, hardware_interface::HW_IF_POSITION, &front_wheel_l_.pos));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    front_wheel_r_.name, hardware_interface::HW_IF_POSITION, &front_wheel_r_.pos));


  state_interfaces.emplace_back(hardware_interface::StateInterface(
    rear_wheel_l_.name, hardware_interface::HW_IF_POSITION, &rear_wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    rear_wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &rear_wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    rear_wheel_r_.name, hardware_interface::HW_IF_POSITION, &rear_wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    rear_wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &rear_wheel_r_.vel));
  

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
{
  /*std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    if (joint.first == "steering")
    {
      RCLCPP_INFO(get_logger(), "STEERING COMMAND  %s  ", joint.second.joint_name.c_str() );
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.second.joint_name, hardware_interface::HW_IF_POSITION,
        &joint.second.command.position));
    }
    else if (joint.first == "traction")
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.second.joint_name, hardware_interface::HW_IF_VELOCITY,
        &joint.second.command.velocity));
    }
  }

  RCLCPP_INFO(get_logger(), "Exported %zu command interfaces.", command_interfaces.size());

  for (auto i = 0u; i < command_interfaces.size(); i++)
  {
    RCLCPP_INFO(
      get_logger(), "Exported command interface '%s'.", command_interfaces[i].get_name().c_str());
  }

  return command_interfaces;*/

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    front_wheel_l_.name, hardware_interface::HW_IF_POSITION, &front_wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    front_wheel_r_.name, hardware_interface::HW_IF_POSITION, &front_wheel_r_.cmd));

  
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    rear_wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &rear_wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    rear_wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &rear_wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.pid_p > 0)
  {
    comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
  /*
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }

  for (auto & joint : hw_interfaces_)
  {
    joint.second.state.position = 0.0;

    if (joint.first == "traction")
    {
      joint.second.state.velocity = 0.0;
      joint.second.command.velocity = 0.0;
    }

    else if (joint.first == "steering")
    {
      joint.second.command.position = 0.0;
    }
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;*/
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduinoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  //comms_.read_encoder_values(front_wheel_l_.enc, front_wheel_r_.enc);

  /*double delta_seconds = period.seconds();

  double pos_prev = front_wheel_l_.pos;
  front_wheel_l_.pos = front_wheel_l_.calc_enc_angle();
  front_wheel_l_.vel = (front_wheel_l_.pos - pos_prev) / delta_seconds;

  pos_prev = front_wheel_r_.pos;
  front_wheel_r_.pos = front_wheel_r_.calc_enc_angle();
  front_wheel_r_.vel = (front_wheel_r_.pos - pos_prev) / delta_seconds;*/

  return hardware_interface::return_type::OK;
}

int count = 0;

hardware_interface::return_type ackerman_robot::DiffDriveArduinoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  
  // rear_wheel_l_.cmd -- velocity -- double, in m/s
  // front_wheel_l_.cmd -- position -- double, in rad

  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }
  
  std::stringstream ss;
  std::stringstream ss2;
  ss2 << "count : " << count << " \n";
  //RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "%s", ss2.str().c_str());

  if (count == 0)
  {

    int rear_wheel_l_counts_per_loop = rear_wheel_l_.cmd / rear_wheel_l_.rads_per_count / cfg_.loop_rate;
    if ( front_wheel_l_.cmd  > 1.50 || front_wheel_l_.cmd < -1.5)
    {
      ss << "front_wheel_l.cmd: " << front_wheel_l_.cmd <<  "front_wheel_l.pos: " << front_wheel_l_.pos <<
      "\nrear_wheel_l_: " << rear_wheel_l_.cmd << "\r";
      //RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "%s", ss.str().c_str());
      comms_.up_servos_values(front_wheel_l_.cmd);
    }
    else if (rear_wheel_l_.cmd  != 0)
    {
      ss << "front_wheel_l.cmd: " << front_wheel_l_.cmd <<  "front_wheel_l.pos: " << front_wheel_l_.pos <<
      "\nrear_wheel_l_: " << rear_wheel_l_.cmd << "\r";
     // RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "%s", ss.str().c_str());
      comms_.up_servos_values(front_wheel_l_.cmd);
    }
    comms_.set_brush_values(rear_wheel_l_counts_per_loop);
  }

  count = (count + 1) % 5;
  
  

  
  
  //ss << "servo_l_count = " << rear_wheel_l_counts_per_loop << "\n";
  //RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "%s", ss.str().c_str());

 
  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ackerman_robot::DiffDriveArduinoHardware, hardware_interface::SystemInterface)