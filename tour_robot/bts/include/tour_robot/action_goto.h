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
#ifndef BT_UC_TOUR_ROBOT_ACTION_GOTO_H
#define BT_UC_TOUR_ROBOT_ACTION_GOTO_H

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <ros2_bt_utils/action_client.hpp>

using NavigateToPoseFeedback = typename rclcpp_action::ClientGoalHandle<
  nav2_msgs::action::NavigateToPose>::Feedback::ConstSharedPtr;

class ActionGoto : public ros2_bt_utils::ActionROSActionClient<nav2_msgs::action::NavigateToPose>
{
public:
  ActionGoto(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts() { return {BT::InputPort<geometry_msgs::msg::Pose>("pose")}; }

private:
  nav2_msgs::action::NavigateToPose::Goal computeGoal() override;
  void elaborateFeedback(const ROSActionGoalFeedbackConstPtr feedback) override;
  BT::NodeStatus elaborateResultAndReturn(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult result)
    override;
  rclcpp::Node::SharedPtr ros_node_;
};
#endif  // BT_UC_TOUR_ROBOT_ACTION_GOTO_H
