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
#include <tour_robot/action_write_current_poi_on_bb.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ros2_bt_utils/action_ros_service_client.hpp>
#include <tour_interfaces/srv/get_current_poi.hpp>

/*Action node that takes the current POI from the service tour_interfaces::srv::GetCurrentPOI and
 * writes it on the blackboard*/
ActionWriteCurrentPOIOnBB::ActionWriteCurrentPOIOnBB(
  const std::string & name, const BT::NodeConfiguration & config)
: ActionROSServiceClient(name, config, "get_current_poi")
{
  RCLCPP_INFO(ros2_bt_utils::ROSNode()->get_logger(), "Creating %s", name.c_str());
}

BT::NodeStatus ActionWriteCurrentPOIOnBB::elaborateResponseAndReturn(
  tour_interfaces::srv::GetCurrentPOI::Response::SharedPtr result)
{
  if (!result->is_ok) {
    RCLCPP_ERROR(ros2_bt_utils::ROSNode()->get_logger(), "Something went wrong in service result");
    return BT::NodeStatus::FAILURE;
  }
  setOutput("poi_pose", result.get()->pose);
  setOutput("poi_description", result.get()->description.c_str());
  setOutput("poi_label", result.get()->label.c_str());
  return BT::NodeStatus::SUCCESS;
}

tour_interfaces::srv::GetCurrentPOI::Request::SharedPtr ActionWriteCurrentPOIOnBB::computeRequest()
{
  // Empty request
  auto request = std::make_shared<tour_interfaces::srv::GetCurrentPOI::Request>();
  return request;
}
