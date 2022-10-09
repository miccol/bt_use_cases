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

#ifndef BT_UC_TOUR_ROBOT_ACTION_WRITE_CURRENT_POI_ON_BB_H
#define BT_UC_TOUR_ROBOT_ACTION_WRITE_CURRENT_POI_ON_BB_H

#include <behaviortree_cpp_v3/action_node.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ros2_bt_utils/action_ros_service_client.hpp>
#include <tour_interfaces/srv/get_current_poi.hpp>

class ActionWriteCurrentPOIOnBB
: public ros2_bt_utils::ActionROSServiceClient<tour_interfaces::srv::GetCurrentPOI>
{
public:
  ActionWriteCurrentPOIOnBB(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::Pose>("poi_pose"),
      BT::OutputPort<std::string>("poi_label"), BT::OutputPort<std::string>("poi_description")};
  }

  BT::NodeStatus elaborateResponseAndReturn(
    tour_interfaces::srv::GetCurrentPOI::Response::SharedPtr result) override;

  tour_interfaces::srv::GetCurrentPOI::Request::SharedPtr computeRequest() override;
};

#endif /* BT_UC_TOUR_ROBOT_ACTION_WRITE_CURRENT_POI_ON_BB_H */
