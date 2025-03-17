/**
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
#ifndef WIFERION_CHARGER_DRIVER_H
#define WIFERION_CHARGER_DRIVER_H

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
        std::array<unsigned char, 8> data_;
        Frame(): available_(false){};
        void printData();
      };

      class ChargerStatus: public Frame
      {
      public:
        struct Field{
          // Gain: 0.1 V, Offset: 0, Signed (Example: 456 = 45.6 V)
          unsigned char output_voltage_high;
          unsigned char output_voltage_low;
          // Gain: 0.1 A, Offset: 0, Signed (Example: 321 = 32.1 A)
          unsigned char output_current_high;
          unsigned char output_current_low;
          // Reserved Bytes (4 - 6)
          unsigned char reserved_0;
          unsigned char reserved_1;
          unsigned char reserved_2;
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
          unsigned char charger_state;
        };
        Field field_;

        struct Values{
          float output_voltage;
          float output_current;
          unsigned char charger_state;
        };

        ChargerStatus(): Frame() {};
        Values getValues();
      };

      class ChargerControl: public Frame
      {
      public:
        struct Field
        {
          // Maximum battery voltage.
          // Gain: 0.1, Offset: 0, Signed (Example: 552 = 55.2 V)
          unsigned char max_voltage_high;
          unsigned char max_voltage_low;
          // Reference current
          // Gain: 0.1, Offset: 0, Signed (Example: 601 = 60.1 A)
          unsigned char ref_current_high;
          unsigned char ref_current_low;
          // Reference voltage
          // Gain: 0.1, Offset: 0, Signed.
          // If set to a non-zero value, the CV phase will be performed by the charger.
          unsigned char ref_voltage_high;
          unsigned char ref_voltage_low;
          // Reserved
          unsigned char reserved_0;
          unsigned char reserved_1;
        };
        Field field_;

        struct Values{
          float max_voltage;
          float ref_current;
          float ref_voltage;
        };

        ChargerControl(): Frame() {};
        Values getValues();
      };

      class SerialNumber: public Frame
      {
      public:
        struct Field
        {
          // 32-bit serial number as on ID label
          unsigned char serial_3;
          unsigned char serial_2;
          unsigned char serial_1;
          unsigned char serial_0;
          // Reserved
          unsigned long reserved;
        };
        Field field_;

        struct Values{
          unsigned int serial;
        };

        SerialNumber(): Frame() {};
        Values getValues();
      };

      class HeatsinkTemperature: public Frame
      {
      public:
        struct Field
        {
          // Reserved
          unsigned char reserved_a;
          unsigned char reserved_b;
          unsigned char reserved_c;
          unsigned char reserved_d;
          // Temperature in C
          // Gain: 0.75, Offset: -40, Unsigned.
          // Min: -40 C, Max: 150 C
          // Example 86 = 24.5 C, 0xFF = Invalid Value
          unsigned char temp;
          // Reserved
          unsigned char reserved_0;
          unsigned char reserved_1;
          unsigned char reserved_2;
        };
        Field field_;

        struct Values{
          float heatsink_temperature;
        };

        HeatsinkTemperature(): Frame() {};
        Values getValues();
      };

      class TerminalTemperature: public Frame
      {
      public:
        struct Field
        {
          // Reserved
          unsigned char reserved_0;
          // Mobile coil temperature
          // Gain: 0.75, Offset -40, unsigned.
          // Min: -40 C, Max: 150 C
          unsigned char coil_temp;
          // Reserved
          unsigned char reserved_1;
          // Temperature at HF1 terminal of ME
          unsigned char hf1_temp;
          // Temperature at HF2 terminal of ME
          unsigned char hf2_temp;
          // Temperature at (+) battery terminal
          unsigned char positive_temp;
          // Temperature at (-) battery terminal
          unsigned char negative_temp;
        };
        Field field_;

        struct Values{
          float coil_temperature;
          float hf1_temperature;
          float hf2_temperature;
          float positive_temperature;
          float negative_temperature;
        };

        TerminalTemperature(): Frame() {};
        Values getValues();
      };

      class Error: public Frame
      {
      public:
        struct Field
        {
          // Byte 0: Reserved
          unsigned char reserved_0;
          // Byte 1
          // - bit 3: Charging disabled due to over-temperature
          unsigned char reserved_1:3;
          bool over_temperature:1;
          unsigned char reserved_2:4;
          // Byte 2
          // - bit 0: Communication timeout error
          // - bit 1: Communication CRC error
          // - bit 2 - 4: Reserved
          // - bit 5: Battery temperature out of limits
          // - bit 6: Time exceeded during optional pre-charge phase
          // - bit 7: Voltage and/or temperature errors reported by bms
          bool comm_timeout:1;
          bool comm_crc_error:1;
          unsigned char reserved_3:3;
          bool batt_temp_limit:1;
          bool pre_charge_time_limit:1;
          bool volt_temp_error:1;
          // Byte 3
          // - bit 0: Required CAN message not received in less than 3x msg periods.
          bool can_message:1;
          unsigned char reserved_4:7;
          // Byte 4: Reserved
          unsigned char reserved_5;
          // Byte 5:
          // - bit 0: SE grid error
          bool grid_error:1;
          unsigned char reserved_6:7;
          // Byte 6:
          // - bit 0: Coil not connected to SE
          // - bit 1 - 2: Reserved
          // - bit 3: SE overtemperature
          // - bit 4 - 5: Reserved
          // - bit 6: No charging due to bad coil positioning
          bool se_coil_disconnected:1;
          unsigned char reserved_7:2;
          bool se_over_temperature:1;
          unsigned char reserved_8:2;
          bool bad_coil_position:1;
          unsigned char reserved_9:1;
          // Byte 7;
          // - bit 0: Fan RPM < 3000 RPM
          // - bit 1: Delivered current limited due to temperature derating
          // - bit 2: Charge current limited due to temperature derating
          // - bit 3: Battery current is limited due to poor positioning (too close)
          // - bit 4: Charging disabled via CAN
          // - bit 5: Power dderating due to grid voltage
          // - bit 6: Derating due to max power
          // - bit 7: SE temperature derating
          bool fan_rpm_low:1;
          bool delivered_current_limit:1;
          bool charge_current_limit:1;
          bool batt_current_limit:1;
          bool charging_disabled:1;
          bool power_derating:1;
          bool max_power_derating:1;
          bool temperature_derating:1;
        };
        Field field_;

        struct Values{
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
          bool charging_disabled;
          bool power_derating;
          bool max_power_derating;
          bool temperature_derating;
        };

        Error(): Frame() {};
        Values getValues();
      };

      class Version: public Frame
      {
      public:
        struct Field
        {
          // Revision number (C)
          unsigned char revision_high;
          unsigned char revision_low;
          // Major, Minor
          // Bit 0 - 3: Minor
          // Bit 4 - 7: Major
          unsigned char minor_major;
          // Reserved
          unsigned char reserved_0;
          unsigned long reserved_1;
        };
        Field field_;

        struct Values{
          int revision;
          int minor;
          int major;
        };

        Version(): Frame() {};
        Values getValues();
      };

      struct Config
      {
        // Reference charge current
        // Gain: 0.02 A, Offset: 0, Unsigned (Example 3123 = 62.46 A)
        unsigned char ref_charge_current_high;
        // Reference charge current/voltage nibbles
        unsigned char ref_charge_curr_volt_nibble;
        // Reference charge voltage low
        unsigned char ref_charge_voltage_low;
        // BMS Type
        // 0x0: None
        // 0x1: Generic
        // 0x2: SCiB
        // 0x3: Reserved
        // 0x4: etaSTORE Type B
        // 0xFF: Unkown
        unsigned char bms_type;
        // Reserved
        unsigned long reserved;
      };

      struct StatStatus
      {
        // Reserved
        unsigned long reserved_0;
        unsigned int reserved_1;
        // Grid RMS voltage
        // Gain: 0.01 V, Offset 0, Unsigned (Example 23123 = 231.23 V)
        unsigned char grid_rms_voltage_high;
        unsigned char grid_rms_voltage_low;
      };

      WiferionCharger();
      void processMessage(can_msgs::msg::Frame msg);
      void processChargerStatus(can_msgs::msg::Frame msg);
      void processSerialNumber(can_msgs::msg::Frame msg);
      void processHeatsinkTemperature(can_msgs::msg::Frame msg);
      void processTerminalTemperature(can_msgs::msg::Frame msg);

      void copyData(std::array<unsigned char, 8> data);

      void processFrameData(WiferionCharger::Frame &frame, std::array<unsigned char, 8> data);

      bool debug_;

    private:
      std::queue<can_msgs::msg::Frame> can_queue_;
      // Fields
      ChargerStatus charger_status_;
      SerialNumber serial_number_;
      HeatsinkTemperature heatsink_temperature_;
      TerminalTemperature terminal_temperature_;
      Error error_;
      Version version_;
  };
}

#endif
