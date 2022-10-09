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

#include <behaviortree_cpp_v3/action_node.h>
#include <ros2_bt_utils/ros2_utils.h>
#include <simple_example/action_wait.h>

/*Action node that leaves the robot in idle (is does nothing)*/

ActionWait::ActionWait(const std::string & name, const BT::NodeConfiguration & config)
: BT::CoroActionNode(name, config)
{
  ros_node_ = ros2_bt_utils::ROSNode();
}

BT::NodeStatus ActionWait::tick()
{
  if (!ros_node_) {
    return BT::NodeStatus::FAILURE;
  }

  while (rclcpp::ok()) {
    RCLCPP_INFO(ros_node_->get_logger(), "Waiting");
    setStatusRunningAndYield();
  }
  return BT::NodeStatus::FAILURE;
}
