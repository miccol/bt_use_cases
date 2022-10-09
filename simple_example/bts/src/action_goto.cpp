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

#include <simple_example/action_goto.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <ros2_bt_utils/action_client.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using NavigateToPoseFeedback = typename rclcpp_action::ClientGoalHandle<
  nav2_msgs::action::NavigateToPose>::Feedback::ConstSharedPtr;

/* Action Node that reads a geometry_msgs::msg::Pose from the blackboard and sends the related
 * nav2_msgs::action::NavigateToPose::Goal to Nav2 */

ActionGoto::ActionGoto(const std::string & name, const BT::NodeConfiguration & config)
: ActionROSActionClient(name, config, "navigate_to_pose")
{
  RCLCPP_INFO(ros2_bt_utils::ROSNode()->get_logger(), "Creating %s", name.c_str());
}

nav2_msgs::action::NavigateToPose::Goal ActionGoto::computeGoal()
{
  geometry_msgs::msg::Pose goal_pose;
  getInput("pose", goal_pose);  // get goal value from BT blackboard
  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.pose = goal_pose;
  return goal;
}

void ActionGoto::elaborateFeedback(const ROSActionGoalFeedbackConstPtr feedback)
{
  RCLCPP_INFO(
    ros2_bt_utils::ROSNode()->get_logger(), "ETA: %d",
    feedback.get()->estimated_time_remaining.sec);
}

BT::NodeStatus ActionGoto::elaborateResultAndReturn(
  [[maybe_unused]] const rclcpp_action::ClientGoalHandle<
    nav2_msgs::action::NavigateToPose>::WrappedResult result)
{
  // NavigateToPose.action is std_msgs/Empty
  return BT::NodeStatus::SUCCESS;
}
