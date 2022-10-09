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

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <ros2_bt_utils/ros2_utils.h>
#include <tour_robot/action_goto.h>
#include <tour_robot/action_say_text.h>
#include <tour_robot/action_update_scheduler.h>
#include <tour_robot/action_wait.h>
#include <tour_robot/action_write_current_poi_on_bb.h>
#include <tour_robot/condition_battery_is_not_charging.h>
#include <tour_robot/condition_battery_level_above.h>
#include <tour_robot/condition_is_at.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#ifdef ZMQ_FOUND
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#endif

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = ros2_bt_utils::ROSNode();
  node->declare_parameter("battery", 1.0);
  node->declare_parameter(
    "bt_filename",
    std::string(
      ament_index_cpp::get_package_share_directory("tour_bts") + "/descriptions/tour_bt.xml"));
  rclcpp::Parameter param_bt_filename = node->get_parameter("bt_filename");

  RCLCPP_INFO(
    node->get_logger(), "Loading  BT XML filename : %s",
    param_bt_filename.value_to_string().c_str());
  RCLCPP_INFO(node->get_logger(), "Registering Nodes");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ActionGoto>("GoTo");
  factory.registerNodeType<ActionWait>("Wait");
  factory.registerNodeType<ActionWriteCurrentPOIOnBB>("WriteCurrentPOIOnBB");
  factory.registerNodeType<ActionUpdateScheduler>("UpdateScheduler");
  factory.registerNodeType<ActionSayText>("SayText");

  factory.registerNodeType<ConditionBatteryLevelAbove>("BatteryLevelAbove");
  factory.registerNodeType<ConditionBatteryIsNotCharging>("BatteryIsCharging");
  factory.registerNodeType<ConditionIsAt>("IsAtPose");
  RCLCPP_INFO(node->get_logger(), "Creating Tree");

  auto blackboard = BT::Blackboard::create();
  auto tree = factory.createTreeFromFile(param_bt_filename.value_to_string().c_str(), blackboard);
  RCLCPP_INFO(node->get_logger(), "Created");
  geometry_msgs::msg::Pose charging_station_pose;
  charging_station_pose.position.x = 1.5;
  charging_station_pose.position.y = 1.5;

  charging_station_pose.orientation.z = 0.707;
  charging_station_pose.orientation.w = 0.707;

  blackboard->set("charging_station", charging_station_pose);
  RCLCPP_INFO(node->get_logger(), "Creating loggers");

  // BT::StdCoutLogger logger_cout(tree);

#ifdef ZMQ_FOUND
  // This publish tree to visualize it with Groot
  PublisherZMQ publisher_zmq(tree);
#endif
  BT::FileLogger logger_file(tree, "bt_trace.fbl");

  RCLCPP_INFO(node->get_logger(), "Running BT");

  while (rclcpp::ok()) {
    tree.tickRoot();
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();

  return 0;
}
