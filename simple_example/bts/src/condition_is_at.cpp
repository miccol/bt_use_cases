//
// Copyright (c) 2022 by Michele Colledanchise. All Rights Reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <behaviortree_cpp_v3/condition_node.h>
#include <simple_example/condition_is_at.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ros2_bt_utils/condition_topic_subscriber.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/* Condition Node that reads the topic /amcl_pose and evaluates if the pose is withing a pose of
 * reference. linear and angular theshold values are set in the blackboard*/

ConditionIsAt::ConditionIsAt(const std::string & name, const BT::NodeConfiguration & config)
: ConditionTopicSubscriber(name, config, "/amcl_pose")
{
  RCLCPP_INFO(ros2_bt_utils::ROSNode()->get_logger(), "Creating %s", name.c_str());
}

BT::NodeStatus ConditionIsAt::elaborateMessageAndReturn(
  geometry_msgs::msg::PoseWithCovarianceStamped msg)
{
  double linear_threshold, angular_threshold;
  geometry_msgs::msg::Pose reference_pose;
  getInput("pose", reference_pose);                // changes over time
  getInput("linear_threshold", linear_threshold);  // may change over time (not in this example)
  getInput("angular_threshold", angular_threshold);
  auto linear_distance = std::sqrt(
    std::pow(msg.pose.pose.position.x - reference_pose.position.x, 2) +
    std::pow(msg.pose.pose.position.y - reference_pose.position.y, 2));
  auto angular_distance = std::atan2(
    msg.pose.pose.position.y - reference_pose.position.y,
    msg.pose.pose.position.x - reference_pose.position.x);
  return (linear_distance <= linear_threshold && angular_distance <= angular_threshold)
           ? BT::NodeStatus::SUCCESS
           : BT::NodeStatus::FAILURE;
}

BT::NodeStatus ConditionIsAt::boundaryConditionStatus()
{
  // if no message has arrived. I assume it is not at the pose
  return BT::NodeStatus::FAILURE;
}
