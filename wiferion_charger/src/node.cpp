/**
Software License Agreement (BSD)

\file      wiferion_charger/node.cpp
\authors   Luis Camero <lcamero@clearpathrobotics.com>
\copyright Copyright (c) 2025, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of Clearpath Robotics nor the names of its contributors
  may be used to endorse or promote products derived from this software
  without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/
#include "wiferion_charger/node.hpp"

namespace wiferion_charger
{

WiferionNode::WiferionNode(const std::string node_name)
  : Node(node_name)
{
  // Declare Parameters
  this->declare_parameter("canbus_dev", "vcan0");
  this->declare_parameter("frequency", 10);

  // Get Parameters
  this->get_parameter("canbus_dev", canbus_dev_);
  this->get_parameter("frequency", freq_);

  // Publishers
  pubStatus_ = this->create_publisher<wiferion_interfaces::msg::Status>("~/status", 10);
  pubError_ = this->create_publisher<wiferion_interfaces::msg::Error>("~/error", 10);
  pubState_ = this->create_publisher<wiferion_interfaces::msg::MobileState>("~/mobile_state", 10);
  pubStatState_ =
    this->create_publisher<wiferion_interfaces::msg::StationaryState>("~/stationary_state", 10);

  // Subscribers
  subDisable_ = this->create_subscription<std_msgs::msg::Bool>(
                  "~/disable_charging",
                  10,
                  std::bind(&WiferionNode::subDisableCallback, this, std::placeholders::_1));

  // Initialize Variables
  recv_msg_.reset(new can_msgs::msg::Frame());

  // Node handle
  node_handle_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

  // Socket
  interface_.reset(new clearpath_ros2_socketcan_interface::SocketCANInterface(
                     canbus_dev_, node_handle_));

  interface_->startSendTimer(1);

  // Run loop
  run_timer_ = this->create_wall_timer(
                 std::chrono::milliseconds(1000 / freq_), std::bind(&WiferionNode::run, this));
}

void WiferionNode::run()
{
  // Process received messages
  while (interface_->recv(recv_msg_))
  {
    if (recv_msg_->dlc == WIFERION_CAN_DATA_LENGTH)
    {
      wiferion_.processMessage(recv_msg_->id, recv_msg_->data);
    }
  }
  // Charger Status
  if (wiferion_.charger_status_.available_)
  {
    WiferionCharger::ChargerStatus::Values status = wiferion_.charger_status_.getValues();
    wiferion_interfaces::msg::Status msg;
    msg.output_voltage = status.output_voltage;
    msg.output_current = status.output_current;
    msg.state = status.charger_state;
    pubStatus_->publish(msg);
  }
  // Error
  if (wiferion_.error_.available_)
  {
    WiferionCharger::Error::Values errors = wiferion_.error_.getValues();
    wiferion_interfaces::msg::Error msg;
    msg.over_temperature = errors.over_temperature;
    msg.comm_timeout = errors.comm_timeout;
    msg.comm_crc_error = errors.comm_crc_error;
    msg.batt_temp_limit = errors.batt_temp_limit;
    msg.pre_charge_time_limit = errors.pre_charge_time_limit;
    msg.volt_temp_error = errors.volt_temp_error;
    msg.can_message = errors.can_message;
    msg.grid_error = errors.grid_error;
    msg.se_coil_disconnected = errors.se_coil_disconnected;
    msg.se_over_temperature = errors.se_over_temperature;
    msg.bad_coil_position = errors.bad_coil_position;
    msg.fan_rpm_low = errors.fan_rpm_low;
    msg.delivered_current_limit = errors.delivered_current_limit;
    msg.charge_current_limit = errors.charge_current_limit;
    msg.batt_current_limit = errors.batt_current_limit;
    msg.charging_disabled = errors.charging_disabled;
    msg.power_derating = errors.power_derating;
    msg.max_power_derating = errors.max_power_derating;
    msg.temperature_derating = errors.temperature_derating;
    pubError_->publish(msg);
  }
  // Mobile State
  if (wiferion_.version_.available_ &
      wiferion_.serial_number_.available_ &
      wiferion_.heatsink_temperature_.available_ &
      wiferion_.terminal_temperature_.available_ &
      wiferion_.config_.available_)
  {
    wiferion_interfaces::msg::MobileState msg;
    // Version
    WiferionCharger::Version::Values version = wiferion_.version_.getValues();
    msg.version_revision = version.revision;
    msg.version_minor = version.minor;
    msg.version_major = version.major;
    // SerialNumber
    WiferionCharger::SerialNumber::Values serial_number = wiferion_.serial_number_.getValues();
    msg.serial_number = serial_number.serial;
    // Heatsink Temperature
    WiferionCharger::HeatsinkTemperature::Values heatsink_temperature =
      wiferion_.heatsink_temperature_.getValues();
    msg.heatsink_temperature = heatsink_temperature.heatsink_temperature;
    // Terminal Temperature
    WiferionCharger::TerminalTemperature::Values terminal_temperature =
      wiferion_.terminal_temperature_.getValues();
    msg.coil_temperature = terminal_temperature.coil_temperature;
    msg.hf1_temperature = terminal_temperature.hf1_temperature;
    msg.hf2_temperature = terminal_temperature.hf2_temperature;
    msg.positive_temperature = terminal_temperature.positive_temperature;
    msg.negative_temperature = terminal_temperature.negative_temperature;
    // Config
    WiferionCharger::Config::Values config = wiferion_.config_.getValues();
    msg.ref_charge_current = config.ref_charge_current;
    msg.ref_charge_voltage = config.ref_charge_voltage;
    msg.bms_type = config.bms_type;
    // Publish
    pubState_->publish(msg);
  }
  // Stationary State
  if (wiferion_.stat_serial_number_.available_ &
      wiferion_.stat_version_.available_ &
      wiferion_.stat_heatsink_temperature_.available_ &
      wiferion_.stat_coil_temperature_.available_ &
      wiferion_.stat_status_.available_)
  {
    wiferion_interfaces::msg::StationaryState msg;
    // Version
    WiferionCharger::Version::Values version = wiferion_.stat_version_.getValues();
    msg.version_revision = version.revision;
    msg.version_minor = version.minor;
    msg.version_major = version.major;
    // SerialNumber
    WiferionCharger::SerialNumber::Values serial_number = wiferion_.stat_serial_number_.getValues();
    msg.serial_number = serial_number.serial;
    // Heatsink Temperature
    WiferionCharger::StatHeatsinkTemperature::Values heatsink_temperature =
      wiferion_.stat_heatsink_temperature_.getValues();
    msg.heatsink_temperature = heatsink_temperature.heatsink_temperature;
    // Coil Temperature
    WiferionCharger::StatCoilTemperature::Values coil_temperature =
      wiferion_.stat_coil_temperature_.getValues();
    msg.coil_temperature = coil_temperature.coil_temperature;
    // Grid RMS Voltage
    WiferionCharger::StatStatus::Values stat_status = wiferion_.stat_status_.getValues();
    msg.grid_rms_voltage = stat_status.grid_rms_voltage;
    // Publish
    pubStatState_->publish(msg);
  }
}

void WiferionNode::subDisableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  can_msgs::msg::Frame can_msg;
  can_msg.header.frame_id = "can";
  can_msg.is_extended = true;
  can_msg.id = wiferion_.disable_charging_.getMessageID();
  can_msg.data = wiferion_.disable_charging_.getMessageData(msg->data);
  can_msg.dlc = sizeof(can_msg.data);
  interface_->queue(can_msg);
}

}  // namespace wiferion_charger

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exe;

  std::shared_ptr<wiferion_charger::WiferionNode> wiferion_node =
    std::make_shared<wiferion_charger::WiferionNode>("wiferion_node");

  exe.add_node(wiferion_node);
  exe.spin();

  rclcpp::shutdown();
  return 0;
}
