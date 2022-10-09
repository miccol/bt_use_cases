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
#include <yaml-cpp/yaml.h>  // from yaml_cpp_vendor package

#include <algorithm>  // std::min
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tour_interfaces/srv/get_current_poi.hpp>
#include <tour_interfaces/srv/reset_tour.hpp>
#include <tour_interfaces/srv/update_poi.hpp>
#include <vector>

struct POI
{
  geometry_msgs::msg::Pose pose;
  std::string description;
  std::string label;
};

std::shared_ptr<rclcpp::Node> node;
std::vector<POI> pois;
std::atomic<unsigned int> poi_index{0};

// updates the current poi. In this simple scheduler it circulates around pois.
void updatePOI(
  [[maybe_unused]] const std::shared_ptr<tour_interfaces::srv::UpdatePOI::Request> request,
  std::shared_ptr<tour_interfaces::srv::UpdatePOI::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request updatePOI");
  if (poi_index + 1 == pois.size()) {
    poi_index = 0;  // reset tour
  } else {
    poi_index++;
  }
  response->is_ok = true;
}

void getCurrentPOI(
  [[maybe_unused]] const std::shared_ptr<tour_interfaces::srv::GetCurrentPOI::Request> request,
  std::shared_ptr<tour_interfaces::srv::GetCurrentPOI::Response> response)
{
  RCLCPP_INFO(node->get_logger(), "Incoming request getCurrentPOI");
  response->pose = pois[poi_index].pose;
  response->description = pois[poi_index].description;
  response->is_ok = true;
}

void resetTour(
  [[maybe_unused]] const std::shared_ptr<tour_interfaces::srv::ResetTour::Request> request,
  std::shared_ptr<tour_interfaces::srv::ResetTour::Response> response)
{
  RCLCPP_INFO(node->get_logger(), "Incoming request resetTour");
  poi_index = 0;
  response->is_ok = true;
}

std::vector<POI> load_pois_from_file(std::string map_filename, std::string description_filename)
{
  std::map<std::string, geometry_msgs::msg::Pose>
    pois_map_pose{};                         // map as I want to quickly access value
  geometry_msgs::msg::Pose loaded_poi_pose;  // vector as I want to keep the order

  YAML::Node map_config_file = YAML::LoadFile(map_filename.c_str());
  YAML::Node description_config_file = YAML::LoadFile(description_filename.c_str());

  YAML::Node loaded_yaml_poses = std::move(map_config_file["pois"]);
  YAML::Node loaded_yaml_descriptions = std::move(description_config_file["pois"]);

  std::vector<POI> loaded_pois;

  for (YAML::const_iterator it = loaded_yaml_poses.begin(); it != loaded_yaml_poses.end(); ++it) {
    auto poi_label = it->second["label"].as<std::string>();
    auto loaded_pose = it->second["pose"];

    loaded_poi_pose.position.x = loaded_pose[0][0].as<double>();
    loaded_poi_pose.position.y = loaded_pose[0][1].as<double>();

    loaded_poi_pose.orientation.z = loaded_pose[1][0].as<double>();
    loaded_poi_pose.orientation.w = loaded_pose[1][3].as<double>();

    RCLCPP_INFO(
      node->get_logger(), "Pose Found x: %f y: %f with label %s", loaded_poi_pose.position.x,
      loaded_poi_pose.position.y, poi_label.c_str());

    pois_map_pose[poi_label] = loaded_poi_pose;
  }

  for (YAML::const_iterator it = loaded_yaml_descriptions.begin();
       it != loaded_yaml_descriptions.end(); ++it) {
    auto is_active = it->second["active"].as<bool>();
    if (!is_active) {
      continue;
    }

    POI poi;
    poi.label = it->second["label"].as<std::string>();
    poi.description = it->second["description"].as<std::string>();
    if (pois_map_pose.count(poi.label)) {
      poi.pose = pois_map_pose[poi.label];
      loaded_pois.push_back(poi);
    } else {
      RCLCPP_WARN(
        node->get_logger(), " No pose Found with label %s. Excluding it from the tour.",
        poi.label.c_str());
    }
  }
  return loaded_pois;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("simple_tour_scheduler_server");
  pois = load_pois_from_file(
    ament_index_cpp::get_package_share_directory("tour_components") + "/config/pois_map.yaml",
    ament_index_cpp::get_package_share_directory("tour_components") +
      "/config/pois_descriptions.yaml");

  auto get_current_poi_srv =
    node->create_service<tour_interfaces::srv::GetCurrentPOI>("get_current_poi", &getCurrentPOI);

  auto update_poi_srv =
    node->create_service<tour_interfaces::srv::UpdatePOI>("update_poi", &updatePOI);

  auto reset_tour_srv =
    node->create_service<tour_interfaces::srv::ResetTour>("reset_tour", &resetTour);

  RCLCPP_INFO(rclcpp::get_logger("simple tour scheduler"), "Ready to get requests.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
