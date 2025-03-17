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

  // Initialize Variables
  recv_msg_.reset(new can_msgs::msg::Frame());

  // Node handle
  node_handle_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *){});

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
    wiferion_.processMessage(*recv_msg_);
  }
}

}

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
