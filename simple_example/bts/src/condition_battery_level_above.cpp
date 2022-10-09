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

#include <simple_example/condition_battery_level_above.h>

#include <ros2_bt_utils/condition_topic_subscriber.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

/* Condition Node that reads the topic /battery_state and evaluates it the battery percentage is above
 * a reference value. The reference value is given through the backboard*/

ConditionBatteryLevelAbove::ConditionBatteryLevelAbove(
  const std::string & name, const BT::NodeConfiguration & config)
: ConditionTopicSubscriber(name, config, "/battery_state")
{
  RCLCPP_INFO(ros2_bt_utils::ROSNode()->get_logger(), "Creating %s", name.c_str());
}

BT::NodeStatus ConditionBatteryLevelAbove::elaborateMessageAndReturn(
  sensor_msgs::msg::BatteryState msg)
{
  current_value = msg.percentage;
  getInput("reference_value", reference_value);

  return current_value > reference_value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus ConditionBatteryLevelAbove::boundaryConditionStatus()
{
  // if no message has arrived. I assume that the battery level is high enough
  RCLCPP_WARN(ros2_bt_utils::ROSNode()->get_logger(), "Assuming Battery level high ");
  return BT::NodeStatus::SUCCESS;
}
