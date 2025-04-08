/**
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  \file       driver.hpp
 *  \brief      Wiferion Charger driver
 *  \author     Luis Camero <lcamero@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2024, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */
#ifndef WIFERION_CHARGER__DRIVER_HPP_
#define WIFERION_CHARGER__DRIVER_HPP_

#define WIFERION_DEBUG false
#define WIFERION_CAN_DATA_LENGTH 8

#define WIFERION_MOB_STATUS_CHARGER_ID 0x18FF0000
#define WIFERION_MOB_STATUS_CHARGER_LOWER_ID 0x50E5
#define WIFERION_MOB_STATUS_CHARGER 0x18FF50E5
#define WIFERION_BMS_CHARGER_CONTROL 0x1806E5F4
#define WIFERION_BMS_DISABLE_CHARGING 0x1806E6F4

#define WIFERION_MOB_ID 0x1FFD0000
#define WIFERION_MOB_ID_MASK 0xFFFF0000

#define WIFERION_MOB_LOWER_ID_MASK 0xFF
#define WIFERION_MOB_SN 0x01
#define WIFERION_MOB_TEMP 0x02
#define WIFERION_MOB_TEMP_2 0x03
#define WIFERION_MOB_ERROR 0x04
#define WIFERION_MOB_STAT_SN 0x06
#define WIFERION_MOB_SW 0x07
#define WIFERION_MOB_CONFIG 0x09
#define WIFERION_MOB_STAT_STATUS 0x10
#define WIFERION_MOB_STAT_SW 0x11
#define WIFERION_MOB_STAT_TEMP 0x13
#define WIFERION_MOB_STAT_TEMP_2 0x14
#define WIFERION_CHARGING_DISABLED 0xAA
#define WIFERION_CHARGING_ENABLED 0x00

#include <queue>

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>


namespace wiferion_charger
{
class WiferionCharger
{
public:
  class Frame
  {
  public:
    bool available_;
    bool debug_;
    std::array<uint8_t, 8> data_;
    Frame(): available_(false), debug_(true) {}
    void printData();
    float convertTemperature(uint8_t temperature);
  };

  class ChargerStatus: public Frame
  {
  public:
    struct Field
    {
      // Gain: 0.1 V, Offset: 0, Signed (Example: 456 = 45.6 V)
      uint8_t output_voltage_high;
      uint8_t output_voltage_low;
      // Gain: 0.1 A, Offset: 0, Signed (Example: 321 = 32.1 A)
      uint8_t output_current_high;
      uint8_t output_current_low;
      // Reserved Bytes (4 - 6)
      uint8_t reserved_0;
      uint8_t reserved_1;
      uint8_t reserved_2;
      // Charger State (4 Bits).
      // 0: Idle
      // 1: Charging
      // 2: Charging constant current
      // 3: Charging constant voltage
      // 4: Charging pre-charge
      // 5: Charging second constant voltage
      // 6: Battery full
      // 7: Stopped
      // 8: Error
      uint8_t charger_state;
    };
    Field field_;

    struct Values
    {
      float output_voltage;
      float output_current;
      uint8_t charger_state;
    };

    ChargerStatus(): Frame() {}
    Values getValues();
  };

  class ChargerControl: public Frame
  {
  public:
    struct Field
    {
      // Maximum battery voltage.
      // Gain: 0.1, Offset: 0, Signed (Example: 552 = 55.2 V)
      uint8_t max_voltage_high;
      uint8_t max_voltage_low;
      // Reference current
      // Gain: 0.1, Offset: 0, Signed (Example: 601 = 60.1 A)
      uint8_t ref_current_high;
      uint8_t ref_current_low;
      // Reference voltage
      // Gain: 0.1, Offset: 0, Signed.
      // If set to a non-zero value, the CV phase will be performed by the charger.
      uint8_t ref_voltage_high;
      uint8_t ref_voltage_low;
      // Reserved
      uint8_t reserved_0;
      uint8_t reserved_1;
    };
    Field field_;

    struct Values
    {
      float max_voltage;
      float ref_current;
      float ref_voltage;
    };

    ChargerControl(): Frame() {}
    Values getValues();
  };

  class SerialNumber: public Frame
  {
  public:
    struct Field
    {
      // 32-bit serial number as on ID label
      uint8_t serial_3;
      uint8_t serial_2;
      uint8_t serial_1;
      uint8_t serial_0;
      // Reserved
      uint16_t reserved;
    };
    Field field_;

    struct Values
    {
      uint32_t serial;
    };

    SerialNumber(): Frame() {}
    Values getValues();
  };

  class HeatsinkTemperature: public Frame
  {
  public:
    struct Field
    {
      // Reserved
      uint8_t reserved_a;
      uint8_t reserved_b;
      uint8_t reserved_c;
      uint8_t reserved_d;
      // Temperature in C
      // Gain: 0.75, Offset: -40, Unsigned.
      // Min: -40 C, Max: 150 C
      // Example 86 = 24.5 C, 0xFF = Invalid Value
      uint8_t temp;
      // Reserved
      uint8_t reserved_0;
      uint8_t reserved_1;
      uint8_t reserved_2;
    };
    Field field_;

    struct Values
    {
      float heatsink_temperature;
    };

    HeatsinkTemperature(): Frame() {}
    Values getValues();
  };

  class TerminalTemperature: public Frame
  {
  public:
    struct Field
    {
      // Reserved
      uint8_t reserved_0;
      // Mobile coil temperature
      // Gain: 0.75, Offset -40, unsigned.
      // Min: -40 C, Max: 150 C
      uint8_t coil_temp;
      // Reserved
      uint8_t reserved_1;
      // Temperature at HF1 terminal of ME
      uint8_t hf1_temp;
      // Temperature at HF2 terminal of ME
      uint8_t hf2_temp;
      // Temperature at (+) battery terminal
      uint8_t positive_temp;
      // Temperature at (-) battery terminal
      uint8_t negative_temp;
    };
    Field field_;

    struct Values
    {
      float coil_temperature;
      float hf1_temperature;
      float hf2_temperature;
      float positive_temperature;
      float negative_temperature;
    };

    TerminalTemperature(): Frame() {}
    Values getValues();
  };

  class Error: public Frame
  {
  public:
    struct Field
    {
      // Byte 0: Reserved
      uint8_t reserved_0;
      // Byte 1
      // - bit 3: Charging disabled due to over-temperature
      uint8_t reserved_1: 3;
      bool over_temperature: 1;
      uint8_t reserved_2: 4;
      // Byte 2
      // - bit 0: Communication timeout error
      // - bit 1: Communication CRC error
      // - bit 2 - 4: Reserved
      // - bit 5: Battery temperature out of limits
      // - bit 6: Time exceeded during optional pre-charge phase
      // - bit 7: Voltage and/or temperature errors reported by bms
      bool comm_timeout: 1;
      bool comm_crc_error: 1;
      uint8_t reserved_3: 3;
      bool batt_temp_limit: 1;
      bool pre_charge_time_limit: 1;
      bool volt_temp_error: 1;
      // Byte 3
      // - bit 0: Required CAN message not received in less than 3x msg periods.
      bool can_message: 1;
      uint8_t reserved_4: 7;
      // Byte 4: Reserved
      uint8_t reserved_5;
      // Byte 5:
      // - bit 0: SE grid error
      bool grid_error: 1;
      uint8_t reserved_6: 7;
      // Byte 6:
      // - bit 0: Coil not connected to SE
      // - bit 1 - 2: Reserved
      // - bit 3: SE overtemperature
      // - bit 4 - 5: Reserved
      // - bit 6: No charging due to bad coil positioning
      bool se_coil_disconnected: 1;
      uint8_t reserved_7: 2;
      bool se_over_temperature: 1;
      uint8_t reserved_8: 2;
      bool bad_coil_position: 1;
      uint8_t reserved_9: 1;
      // Byte 7;
      // - bit 0: Fan RPM < 3000 RPM
      // - bit 1: Delivered current limited due to temperature derating
      // - bit 2: Charge current limited due to temperature derating
      // - bit 3: Battery current is limited due to poor positioning (too close)
      // - bit 4: Charging disabled via CAN
      // - bit 5: Power dderating due to grid voltage
      // - bit 6: Derating due to max power
      // - bit 7: SE temperature derating
      bool fan_rpm_low: 1;
      bool delivered_current_limit: 1;
      bool charge_current_limit: 1;
      bool batt_current_limit: 1;
      bool charging_disabled: 1;
      bool power_derating: 1;
      bool max_power_derating: 1;
      bool temperature_derating: 1;
    };
    Field field_;

    struct Values
    {
      bool over_temperature;
      bool comm_timeout;
      bool comm_crc_error;
      bool batt_temp_limit;
      bool pre_charge_time_limit;
      bool volt_temp_error;
      bool can_message;
      bool grid_error;
      bool se_coil_disconnected;
      bool se_over_temperature;
      bool bad_coil_position;
      bool fan_rpm_low;
      bool delivered_current_limit;
      bool charge_current_limit;
      bool batt_current_limit;
      bool charging_disabled;
      bool power_derating;
      bool max_power_derating;
      bool temperature_derating;
    };

    Error(): Frame() {}
    Values getValues();
  };

  class Version: public Frame
  {
  public:
    struct Field
    {
      // Revision number (C)
      uint8_t revision_high;
      uint8_t revision_low;
      // Major, Minor
      // Bit 0 - 3: Minor
      // Bit 4 - 7: Major
      uint8_t minor_major;
      // Reserved
      uint8_t reserved_0;
      uint32_t reserved_1;
    };
    Field field_;

    struct Values
    {
      unsigned int revision;
      unsigned int minor;
      unsigned int major;
    };

    Version(): Frame() {}
    Values getValues();
  };

  class Config: public Frame
  {
  public:
    struct Field
    {
      // Reference charge current
      // Gain: 0.02 A, Offset: 0, Unsigned (Example 3123 = 62.46 A)
      uint8_t ref_charge_current_high;
      // Reference charge current/voltage nibbles
      uint8_t ref_charge_volt_nibble: 4;
      uint8_t ref_charge_curr_nibble: 4;
      // Reference charge voltage low
      uint8_t ref_charge_voltage_low;
      // BMS Type
      // 0x0: None
      // 0x1: Generic
      // 0x2: SCiB
      // 0x3: Reserved
      // 0x4: etaSTORE Type B
      // 0xFF: Unknown
      uint8_t bms_type;
      // Reserved
      uint32_t reserved;
    };
    Field field_;

    struct Values
    {
      float ref_charge_current;
      float ref_charge_voltage;
      uint8_t bms_type;
    };

    Config(): Frame() {}
    Values getValues();
  };

  class StatHeatsinkTemperature: public Frame
  {
  public:
    struct Field
    {
      // Reserved
      uint8_t reserved_0;
      uint8_t reserved_1;
      uint8_t reserved_2;
      uint8_t reserved_3;
      uint8_t reserved_4;
      uint8_t reserved_5;
      // Temperature in C
      // Gain: 0.75, Offset: -40, Unsigned.
      // Min: -40 C, Max: 150 C
      // Example 86 = 24.5 C, 0xFF = Invalid Value
      uint8_t temp;
      // Reserved
      uint8_t reserved_6;
    };
    Field field_;

    struct Values
    {
      float heatsink_temperature;
    };

    StatHeatsinkTemperature(): Frame() {}
    Values getValues();
  };

  class StatCoilTemperature: public Frame
  {
  public:
    struct Field
    {
      // Temperature in C
      // Gain: 0.75, Offset: -40, Unsigned.
      // Min: -40 C, Max: 150 C
      // Example 86 = 24.5 C, 0xFF = Invalid Value
      uint8_t temp;
      // Reserved
      uint8_t reserved_0;
      uint8_t reserved_1;
      uint8_t reserved_2;
      uint8_t reserved_3;
      uint8_t reserved_4;
      uint8_t reserved_5;
      uint8_t reserved_6;
    };
    Field field_;

    struct Values
    {
      float coil_temperature;
    };

    StatCoilTemperature(): Frame() {}
    Values getValues();
  };

  class StatStatus: public Frame
  {
  public:
    struct Field
    {
      // Reserved
      uint8_t reserved_0;
      uint8_t reserved_1;
      uint8_t reserved_2;
      uint8_t reserved_3;
      uint8_t reserved_4;
      uint8_t reserved_5;
      // Grid RMS voltage
      // Gain: 0.01 V, Offset 0, unsigned
      uint8_t grid_rms_voltage_high;
      uint8_t grid_rms_voltage_low;
    };
    Field field_;

    struct Values
    {
      float grid_rms_voltage;
    };

    StatStatus(): Frame() {}
    Values getValues();
  };

  class DisableCharging: public Frame
  {
  public:
    struct Field
    {
      // Signature
      // send 0xAA to disable charging
      uint8_t signature;
      uint8_t reserved_0;
      uint16_t reserved_1;
      uint32_t reserved_2;
    };
    Field field_;

    struct Values
    {
      bool charging_disabled;
    };
    DisableCharging(): Frame() {}
    Values getValues();
    std::array<uint8_t, WIFERION_CAN_DATA_LENGTH> getMessageData(bool disable_charging);
    uint32_t getMessageID();
  };

  WiferionCharger();
  void copyData(std::array<uint8_t, WIFERION_CAN_DATA_LENGTH> data);
  void processMessage(uint32_t id, std::array<uint8_t, WIFERION_CAN_DATA_LENGTH> data);
  void processFrameData(WiferionCharger::Frame &frame,
      std::array<uint8_t, WIFERION_CAN_DATA_LENGTH> data);

  bool debug_;

  // Fields
  ChargerStatus charger_status_;
  SerialNumber serial_number_;
  HeatsinkTemperature heatsink_temperature_;
  TerminalTemperature terminal_temperature_;
  Error error_;
  Version version_;
  Config config_;
  SerialNumber stat_serial_number_;
  Version stat_version_;
  StatHeatsinkTemperature stat_heatsink_temperature_;
  StatCoilTemperature stat_coil_temperature_;
  StatStatus stat_status_;

  // Commands
  DisableCharging disable_charging_;
};

}  // namespace wiferion_charger

#endif  // WIFERION_CHARGER__DRIVER_HPP_
