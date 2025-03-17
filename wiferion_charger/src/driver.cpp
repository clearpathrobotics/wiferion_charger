/**
Software License Agreement (BSD)

\file      driver.cpp
\authors   Luis Camero <lcamero@clearpathrobotics.com>
\copyright Copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.

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

#include "wiferion_charger/driver.hpp"

namespace wiferion_charger
{

WiferionCharger::WiferionCharger()
{
  debug_ = true;
}

void WiferionCharger::processMessage(can_msgs::msg::Frame msg)
{
  WiferionCharger::Frame * frame = nullptr;
  // Message request. No data to process return
  if (msg.dlc == 0)
  {
    return;
  }

  // Mask the message ID
  int masked_id = msg.id & WIFERION_MOB_ID_MASK;
  int mob_id = msg.id & WIFERION_MOB_LOWER_ID_MASK;

  // std::cout << "Message ID: " << msg.id << std::endl;

  // Select appropriate fields to parse
  switch (masked_id)
  {
    case WIFERION_MOB_STATUS_CHARGER_ID:
      if((msg.id & 0xFFFF) == WIFERION_MOB_STATUS_CHARGER_LOWER_ID)
      {
        frame = &charger_status_;
      }
      break;
    case WIFERION_MOB_ID:
      // std::cout << "Message MOB: " << std::hex << std::setfill('0') << std::setw(2) << int(msg.id) << std::endl;
      switch (mob_id)
      {
        case WIFERION_MOB_SN:
          frame = &serial_number_;
          break;
        case WIFERION_MOB_TEMP:
          frame = &heatsink_temperature_;
          break;
        case WIFERION_MOB_TEMP_2:
          frame = &terminal_temperature_;
          break;
        case WIFERION_MOB_ERROR:
          frame = &error_;
          break;
        case WIFERION_MOB_STAT_SN:
          break;
        case WIFERION_MOB_SW:
          frame = &version_;
          break;
        case WIFERION_MOB_CONFIG:
          break;
        case WIFERION_MOB_STAT_STATUS:
          break;
        case WIFERION_MOB_STAT_SW:
          break;
        case WIFERION_MOB_STAT_TEMP:
          break;
        case WIFERION_MOB_STAT_TEMP_2:
          break;
      }
      break;
  }
  // Process
  if(frame != nullptr)
  {
    std::cout << "Message ID: " << std::hex << msg.id << std::endl;
    processFrameData(*frame, msg.data);
  }

  if(charger_status_.available_)
  {
    charger_status_.getValues();
  }
  if(serial_number_.available_)
  {
    serial_number_.getValues();
  }
  if(heatsink_temperature_.available_)
  {
    heatsink_temperature_.getValues();
  }
  if(terminal_temperature_.available_)
  {
    terminal_temperature_.getValues();
  }
  // if(error_.available_)
  // {
  //   error_.getValues();
  // }
  if(version_.available_)
  {
    version_.getValues();
  }
}

void WiferionCharger::processFrameData(WiferionCharger::Frame &frame, std::array<unsigned char, 8> data)
{
  std::memcpy(&frame.data_, &data, WIFERION_CAN_DATA_LENGTH);
  frame.available_ = true;
}

void WiferionCharger::Frame::printData()
{
  for (int i = 0; i < WIFERION_CAN_DATA_LENGTH; i++)
    {
      std::cout << std::hex << std::setfill('0') << std::setw(2) << int(data_[i]) << " ";
    }
  std::cout << std::endl;
}

WiferionCharger::ChargerStatus::Values WiferionCharger::ChargerStatus::getValues()
{
  available_ = false;
  // Copy
  std::memcpy(&field_, &data_, WIFERION_CAN_DATA_LENGTH);
  // Reinterpret
  signed short output_voltage = (field_.output_voltage_high << 8) | field_.output_voltage_low;
  signed short output_current = (field_.output_current_high << 8) | field_.output_current_low;
  // Retrieve Values
  WiferionCharger::ChargerStatus::Values values;
  values.output_voltage = 0.1 * output_voltage;
  values.output_current = 0.1 * output_current;
  values.charger_state = field_.charger_state;
  // Debug Log
  if(true)
  {
    std::cout << std::endl << "WiferionCharger::ChargerStatus::Values: " << std::endl;
    std::cout << "Output Voltage: " << values.output_voltage << std::endl;
    std::cout << "Output Current: " << values.output_current << std::endl;
    std::cout << "Charger State: " << int(values.charger_state) << std::endl;
    printData();
    std::cout << std::endl;
  }
  return values;
}

WiferionCharger::SerialNumber::Values WiferionCharger::SerialNumber::getValues()
{
  available_ = false;
  // Copy
  std::memcpy(&field_, &data_, WIFERION_CAN_DATA_LENGTH);
  // Reinterpret and Store
  WiferionCharger::SerialNumber::Values values;
  values.serial = (field_.serial_3 << 24) | (field_.serial_2 << 16) | (field_.serial_1 << 8) | field_.serial_0;
  // Debug Log
  if(true)
  {
    std::cout << "Serial Number: " << std::dec << values.serial << std::endl;
    printData();
    std::cout << std::endl;
  }
  return values;
}

WiferionCharger::HeatsinkTemperature::Values WiferionCharger::HeatsinkTemperature::getValues()
{
  available_ = false;
  // Copy
  std::memcpy(&field_, &data_, WIFERION_CAN_DATA_LENGTH);
  // Reinterpret and Store
  WiferionCharger::HeatsinkTemperature::Values values;
  values.heatsink_temperature = field_.temp * 0.75 - 40;
  // Debug Log
  if(true)
  {
    std::cout << "Heatsink Temperature: " << std::dec << values.heatsink_temperature << std::endl;
    printData();
    std::cout << std::endl;
  }
  return values;
}

WiferionCharger::TerminalTemperature::Values WiferionCharger::TerminalTemperature::getValues()
{
  available_ = false;
  // Copy
  std::memcpy(&field_, &data_, WIFERION_CAN_DATA_LENGTH);
  // Reinterpret and Store
  WiferionCharger::TerminalTemperature::Values values;
  values.coil_temperature = field_.coil_temp * 0.75 - 40;
  values.hf1_temperature = field_.hf1_temp * 0.75 - 40;
  values.hf2_temperature = field_.hf2_temp * 0.75 - 40;
  values.positive_temperature = field_.positive_temp * 0.75 - 40;
  values.negative_temperature = field_.negative_temp * 0.75 - 40;
  // Debug Log
  if(true)
  {
    std::cout << "Coil Temperature: " << std::dec << values.coil_temperature << std::endl;
    std::cout << "HF1 Terminal Temperature: " << std::dec << values.hf1_temperature << std::endl;
    std::cout << "HF2 Terminal Temperature: " << std::dec << values.hf2_temperature << std::endl;
    std::cout << "Positive Battery Terminal Temperature: " << std::dec << values.positive_temperature << std::endl;
    std::cout << "Negative Battery Terminal Temperature: " << std::dec << values.negative_temperature << std::endl;
    printData();
    std::cout << std::endl;
  }
  return values;
}

WiferionCharger::Error::Values WiferionCharger::Error::getValues()
{
  available_ = false;
  // Copy
  std::memcpy(&field_, &data_, WIFERION_CAN_DATA_LENGTH);
  // Store
  WiferionCharger::Error::Values values;
  values.comm_timeout = field_.comm_timeout;
  values.comm_crc_error = field_.comm_crc_error;
  values.batt_temp_limit = field_.batt_temp_limit;
  values.pre_charge_time_limit = field_.pre_charge_time_limit;
  values.volt_temp_error = field_.volt_temp_error;
  values.can_message = field_.can_message;
  values.grid_error = field_.grid_error;
  values.se_coil_disconnected = field_.se_coil_disconnected;
  values.se_over_temperature = field_.se_over_temperature;
  values.bad_coil_position = field_.bad_coil_position;
  values.fan_rpm_low = field_.fan_rpm_low;
  values.delivered_current_limit = field_.delivered_current_limit;
  values.charge_current_limit = field_.charge_current_limit;
  values.charging_disabled = field_.charging_disabled;
  values.power_derating = field_.power_derating;
  values.max_power_derating = field_.max_power_derating;
  values.temperature_derating = field_.temperature_derating;
  // Debug
  if(true)
  {
    std::cout << "comm_timeout: " << values.comm_timeout << std::endl;
    std::cout << "comm_crc_error: " << values.comm_crc_error << std::endl;
    std::cout << "batt_temp_limit: " << values.batt_temp_limit << std::endl;
    std::cout << "pre_charge_time_limit: " << values.pre_charge_time_limit << std::endl;
    std::cout << "volt_temp_error: " << values.volt_temp_error << std::endl;
    std::cout << "can_message: " << values.can_message << std::endl;
    std::cout << "grid_error: " << values.grid_error << std::endl;
    std::cout << "se_coil_disconnected: " << values.se_coil_disconnected << std::endl;
    std::cout << "se_over_temperature: " << values.se_over_temperature << std::endl;
    std::cout << "bad_coil_position: " << values.bad_coil_position << std::endl;
    std::cout << "fan_rpm_low: " << values.fan_rpm_low << std::endl;
    std::cout << "delivered_current_limit: " << values.delivered_current_limit << std::endl;
    std::cout << "charge_current_limit: " << values.charge_current_limit << std::endl;
    std::cout << "charging_disabled: " << values.charging_disabled << std::endl;
    std::cout << "power_derating: " << values.power_derating << std::endl;
    std::cout << "max_power_derating: " << values.max_power_derating << std::endl;
    std::cout << "temperature_derating: " << values.temperature_derating << std::endl;
    printData();
    std::cout << std::endl;
  }
  return values;
}

WiferionCharger::Version::Values WiferionCharger::Version::getValues()
{
  available_ = false;
  // Copy
  std::memcpy(&field_, &data_, WIFERION_CAN_DATA_LENGTH);
  // Re-interpret and Store
  WiferionCharger::Version::Values values;
  values.revision = field_.revision_high << 8 | field_.revision_low;
  values.minor = field_.minor_major & 0x00FF;
  values.major = field_.minor_major >> 8;
  // Debug
  if(true)
  {
    std::cout << "Version: ";
    std::cout << std::dec << values.major << ".";
    std::cout << std::dec << values.minor << ".";
    std::cout << std::dec << values.revision << std::endl;
    printData();
    std::cout << std::endl;
  }
  return values;
}

}
