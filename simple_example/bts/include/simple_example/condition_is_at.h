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

#ifndef BT_UC_SIMPLE_EXAMPLE_CONDITION_IS_AT_H
#define BT_UC_SIMPLE_EXAMPLE_CONDITION_IS_AT_H

#include <behaviortree_cpp_v3/condition_node.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ros2_bt_utils/condition_topic_subscriber.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ConditionIsAt
: public ros2_bt_utils::ConditionTopicSubscriber<geometry_msgs::msg::PoseWithCovarianceStamped>
{
public:
  ConditionIsAt(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::Pose>("pose"), BT::InputPort<double>("linear_threshold"),
      BT::InputPort<double>("angular_threshold")};
  }

  BT::NodeStatus elaborateMessageAndReturn(
    geometry_msgs::msg::PoseWithCovarianceStamped msg) override;
  BT::NodeStatus boundaryConditionStatus() override;
};
#endif /* BT_UC_SIMPLE_EXAMPLE_CONDITION_IS_AT_H */
