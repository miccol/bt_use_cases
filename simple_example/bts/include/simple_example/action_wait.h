/*
 *   Copyright (c) 2022 Michele Colledanchise
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#ifndef BT_UC_SIMPLE_EXAMPLE_ACTION_WAIT_H
#define BT_UC_SIMPLE_EXAMPLE_ACTION_WAIT_H

#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp/rclcpp.hpp>

class ActionWait : public BT::CoroActionNode
{
public:
  ActionWait(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
};

#endif  // BT_UC_SIMPLE_EXAMPLE_ACTION_WAIT_H
