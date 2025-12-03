/**
Software License Agreement (BSD)

\authors   Luis Camero <lcamero@clearpathrobotics.com>
\copyright Copyright (c) 2025, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef WIFERION_CHARGER__NODE_HPP_
#define WIFERION_CHARGER__NODE_HPP_

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "clearpath_ros2_socketcan_interface/socketcan_interface.hpp"
#include "wiferion_charger/driver.hpp"
#include "wiferion_interfaces/msg/status.hpp"
#include "wiferion_interfaces/msg/error.hpp"
#include "wiferion_interfaces/msg/mobile_state.hpp"
#include "wiferion_interfaces/msg/stationary_state.hpp"

namespace wiferion_charger
{

class WiferionNode
  : public rclcpp::Node
{
public:
  explicit WiferionNode(const std::string node_name);
  void run();

private:
  std::string canbus_dev_;
  int freq_;

  WiferionCharger wiferion_;
  std::shared_ptr<clearpath_ros2_socketcan_interface::SocketCANInterface> interface_;
  can_msgs::msg::Frame::SharedPtr recv_msg_;

  rclcpp::Node::SharedPtr node_handle_;
  rclcpp::TimerBase::SharedPtr run_timer_;

  rclcpp::Publisher<wiferion_interfaces::msg::Status>::SharedPtr pubStatus_;
  rclcpp::Publisher<wiferion_interfaces::msg::Error>::SharedPtr pubError_;
  rclcpp::Publisher<wiferion_interfaces::msg::MobileState>::SharedPtr pubState_;
  rclcpp::Publisher<wiferion_interfaces::msg::StationaryState>::SharedPtr pubStatState_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subDisable_;

  void subDisableCallback(const std_msgs::msg::Bool::SharedPtr msg);
};

}  // namespace wiferion_charger

#endif  // WIFERION_CHARGER__NODE_HPP_
