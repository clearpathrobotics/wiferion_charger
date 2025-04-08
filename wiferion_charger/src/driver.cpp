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
  debug_ = WIFERION_DEBUG;
  charger_status_.debug_ = debug_;
  serial_number_.debug_ = debug_;
  heatsink_temperature_.debug_ = debug_;
  terminal_temperature_.debug_ = debug_;
  error_.debug_ = debug_;
  version_.debug_ = debug_;
  config_.debug_ = debug_;
  stat_serial_number_.debug_ = debug_;
  stat_version_.debug_ = debug_;
  stat_heatsink_temperature_.debug_ = debug_;
  stat_coil_temperature_.debug_ = debug_;
  stat_status_.debug_ = debug_;
  disable_charging_.debug_ = debug_;
}

void WiferionCharger::processMessage(uint32_t id,
    std::array<uint8_t, WIFERION_CAN_DATA_LENGTH> data)
{
  WiferionCharger::Frame * frame = nullptr;
  // Mask the message ID
  int masked_id = id & WIFERION_MOB_ID_MASK;
  int mob_id = id & WIFERION_MOB_LOWER_ID_MASK;

  // Select appropriate fields to parse
  switch (masked_id)
  {
  case WIFERION_MOB_STATUS_CHARGER_ID:
    if ((id & 0xFFFF) == WIFERION_MOB_STATUS_CHARGER_LOWER_ID)
    {
      frame = &charger_status_;
    }
    break;
  case WIFERION_MOB_ID:
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
      frame = &stat_serial_number_;
      break;
    case WIFERION_MOB_SW:
      frame = &version_;
      break;
    case WIFERION_MOB_CONFIG:
      frame = &config_;
      break;
    case WIFERION_MOB_STAT_STATUS:
      frame = &stat_status_;
      break;
    case WIFERION_MOB_STAT_SW:
      frame = &stat_version_;
      break;
    case WIFERION_MOB_STAT_TEMP:
      frame = &stat_heatsink_temperature_;
      break;
    case WIFERION_MOB_STAT_TEMP_2:
      frame = &stat_coil_temperature_;
      break;
    }
    break;
  }
  // Process
  if (frame != nullptr)
  {
    if (debug_) std::cout << "Message ID: " << std::hex << id << std::endl;
    processFrameData(*frame, data);
  }
}

void WiferionCharger::processFrameData(WiferionCharger::Frame &frame, std::array<uint8_t, 8> data)
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

float WiferionCharger::Frame::convertTemperature(uint8_t temperature)
{
  if (temperature == 0xFF)
  {
    return std::nanf("NaN");
  }
  else
  {
    return temperature * 0.75 - 40;
  }
}

WiferionCharger::ChargerStatus::Values WiferionCharger::ChargerStatus::getValues()
{
  available_ = false;
  // Copy
  std::memcpy(&field_, &data_, WIFERION_CAN_DATA_LENGTH);
  // Reinterpret
  int16_t output_voltage = (field_.output_voltage_high << 8) | field_.output_voltage_low;
  int16_t output_current = (field_.output_current_high << 8) | field_.output_current_low;
  // Retrieve Values
  WiferionCharger::ChargerStatus::Values values;
  values.output_voltage = 0.1 * output_voltage;
  values.output_current = 0.1 * output_current;
  values.charger_state = field_.charger_state;
  // Debug Log
  if (debug_)
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
  values.serial =
      (field_.serial_3 << 24) | (field_.serial_2 << 16) | (field_.serial_1 << 8) | field_.serial_0;
  // Debug Log
  if (debug_)
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
  values.heatsink_temperature = convertTemperature(field_.temp);
  // Debug Log
  if (debug_)
  {
    std::cout << "Heatsink Temperature: " <<
        std::dec << values.heatsink_temperature << std::endl;
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
  values.coil_temperature = convertTemperature(field_.coil_temp);
  values.hf1_temperature = convertTemperature(field_.hf1_temp);
  values.hf2_temperature = convertTemperature(field_.hf2_temp);
  values.positive_temperature = convertTemperature(field_.positive_temp);
  values.negative_temperature = convertTemperature(field_.negative_temp);
  // Debug Log
  if (debug_)
  {
    std::cout << "Coil Temperature: " << std::dec << values.coil_temperature << std::endl;
    std::cout << "HF1 Terminal Temperature: " << std::dec << values.hf1_temperature << std::endl;
    std::cout << "HF2 Terminal Temperature: " << std::dec << values.hf2_temperature << std::endl;
    std::cout << "Positive Battery Terminal Temperature: " <<
        std::dec << values.positive_temperature << std::endl;
    std::cout << "Negative Battery Terminal Temperature: " <<
        std::dec << values.negative_temperature << std::endl;
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
  values.over_temperature = field_.over_temperature;
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
  values.batt_current_limit = field_.batt_current_limit;
  values.charging_disabled = field_.charging_disabled;
  values.power_derating = field_.power_derating;
  values.max_power_derating = field_.max_power_derating;
  values.temperature_derating = field_.temperature_derating;
  // Debug
  if (debug_)
  {
    std::cout << "over_temperature: " << values.over_temperature << std::endl;
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
    std::cout << "batt_current_limit: " << values.batt_current_limit << std::endl;
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
  if (debug_)
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

WiferionCharger::Config::Values WiferionCharger::Config::getValues()
{
  available_ = false;
  // Copy
  std::memcpy(&field_, &data_, WIFERION_CAN_DATA_LENGTH);
  // Re-interpret and Store
  WiferionCharger::Config::Values values;
  values.ref_charge_current = (
      (field_.ref_charge_current_high << 4) | field_.ref_charge_curr_nibble) * 0.02;
  values.ref_charge_voltage = (
      (field_.ref_charge_volt_nibble << 8) | field_.ref_charge_voltage_low) * 0.02;
  values.bms_type = field_.bms_type;
  // Debug
  if (debug_)
  {
    std::cout << "Reference charge current: " << values.ref_charge_current << std::endl;
    std::cout << "Reference charge voltage: " << values.ref_charge_voltage << std::endl;
    std::cout << "BMS type: " << std::hex << int(values.bms_type) << std::endl;
    printData();
    std::cout << std::endl;
  }
  return values;
}

WiferionCharger::StatHeatsinkTemperature::Values
WiferionCharger::StatHeatsinkTemperature::getValues()
{
  available_ = false;
  // Copy
  std::memcpy(&field_, &data_, WIFERION_CAN_DATA_LENGTH);
  // Re-interpret and store
  WiferionCharger::StatHeatsinkTemperature::Values values;
  values.heatsink_temperature = convertTemperature(field_.temp);
  // Debug Log
  if (debug_)
  {
    std::cout << "Stationary Heatsink Temperature: " <<
        std::dec << values.heatsink_temperature << std::endl;
    printData();
    std::cout << std::endl;
  }
  return values;
}

WiferionCharger::StatCoilTemperature::Values WiferionCharger::StatCoilTemperature::getValues()
{
  available_ = false;
  // Copy
  std::memcpy(&field_, &data_, WIFERION_CAN_DATA_LENGTH);
  // Re-interpret and store
  WiferionCharger::StatCoilTemperature::Values values;
  values.coil_temperature = convertTemperature(field_.temp);
  // Debug Log
  if (debug_)
  {
    std::cout << "Stationary Coil Temperature: " <<
        std::dec << values.coil_temperature << std::endl;
    printData();
    std::cout << std::endl;
  }
  return values;
}

WiferionCharger::StatStatus::Values WiferionCharger::StatStatus::getValues()
{
  available_ = false;
  // Copy
  std::memcpy(&field_, &data_, WIFERION_CAN_DATA_LENGTH);
  // Re-interpret and store
  WiferionCharger::StatStatus::Values values;
  values.grid_rms_voltage =
      0.01 * ((field_.grid_rms_voltage_high << 8) | field_.grid_rms_voltage_low);
  // Debug Log
  if (debug_)
  {
    std::cout << "Stationary Grid RMS Voltage: " <<
        std::dec << values.grid_rms_voltage << std::endl;
    printData();
    std::cout << std::endl;
  }
  return values;
}

WiferionCharger::DisableCharging::Values WiferionCharger::DisableCharging::getValues()
{
  available_ = false;
  // Copy
  std::memcpy(&field_, &data_, WIFERION_CAN_DATA_LENGTH);
  // Re-interpret and store
  WiferionCharger::DisableCharging::Values values;
  values.charging_disabled = WIFERION_CHARGING_DISABLED == field_.signature;
  // Debug Log
  if (debug_)
  {
    std::cout << "Disable Charging: " << int(values.charging_disabled) << std::endl;
    printData();
    std::cout << std::endl;
  }
  return values;
}

std::array<uint8_t, WIFERION_CAN_DATA_LENGTH>
WiferionCharger::DisableCharging::getMessageData(bool disable_charging)
{
  // Create data array
  std::array<uint8_t, WIFERION_CAN_DATA_LENGTH> data;
  // Modify field
  if (disable_charging)
  {
    field_.signature = WIFERION_CHARGING_DISABLED;
  }
  else
  {
    field_.signature = WIFERION_CHARGING_ENABLED;
  }
  // Copy
  std::memcpy(&data, &field_, WIFERION_CAN_DATA_LENGTH);
  return data;
}

uint32_t WiferionCharger::DisableCharging::getMessageID()
{
  return WIFERION_BMS_DISABLE_CHARGING;
}

}  // namespace wiferion_charger
