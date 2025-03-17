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
#ifndef WIFERION_CHARGER_NODE_H
#define WIFERION_CHARGER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include "clearpath_ros2_socketcan_interface/socketcan_interface.hpp"
#include "wiferion_charger/driver.hpp"

namespace wiferion_charger
{

class WiferionNode
  : public rclcpp::Node
{
public:
  WiferionNode(const std::string node_name);
  void run();

private:
  std::string canbus_dev_;
  int freq_;

  WiferionCharger wiferion_;
  std::shared_ptr<clearpath_ros2_socketcan_interface::SocketCANInterface> interface_;
  can_msgs::msg::Frame::SharedPtr recv_msg_;

  rclcpp::Node::SharedPtr node_handle_;
  rclcpp::TimerBase::SharedPtr run_timer_;
};

}

#endif // WIFERION_CHARGER_NODE_H
