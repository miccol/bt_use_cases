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

#include <algorithm>  // std::max
#include <atomic>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <simple_interfaces/srv/charge_battery.hpp>
#include <simple_interfaces/srv/get_battery_state.hpp>
#include <simple_interfaces/srv/set_battery_level.hpp>
#include <vector>

using namespace std::chrono_literals;

/* This component emulates a battery readed component. It publishes a sensor_msgs::msg::BatteryState
 * whose percentage lowers in time unless under charge. It provides the services
 * fake_battery_reader/set_battery_level, fake_battery_reader/charge_battery and
 * fake_battery_reader/charge_battery. While under charge the power_supply_status is
 * POWER_SUPPLY_STATUS_CHARGING, it is POWER_SUPPLY_STATUS_NOT_CHARGING otherwise. Once a chargin
 * battery is full, its supply status becomes POWER_SUPPLY_STATUS_NOT_CHARGING */
class FakeBatteryReader : public rclcpp::Node
{
public:
  FakeBatteryReader() : Node("fake_battery_reader")
  {
    const std::lock_guard<std::mutex> lock(battery_state_mutex_);

    battery_state_.power_supply_status = sensor_msgs::msg::BatteryState::
      POWER_SUPPLY_STATUS_NOT_CHARGING;  // in this example the status toggles between CHARGING
                                         // and NOT_CHARGING
    battery_state_.percentage = 1.0;     // 1.0 means 100%

    battery_state_publisher_ =
      this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 1);
    timer_ = this->create_wall_timer(500ms, std::bind(&FakeBatteryReader::timer_callback, this));

    srv_get_battery_level_ = this->create_service<simple_interfaces::srv::SetBatteryLevel>(
      "fake_battery_reader/set_battery_level",
      std::bind(
        &FakeBatteryReader::set_battery_level, this, std::placeholders::_1, std::placeholders::_2));

    srv_charge_battery_ = this->create_service<simple_interfaces::srv::ChargeBattery>(
      "fake_battery_reader/charge_battery",
      std::bind(
        &FakeBatteryReader::charge_battery, this, std::placeholders::_1, std::placeholders::_2));

    srv_get_battery_state_ = this->create_service<simple_interfaces::srv::GetBatteryState>(
      "fake_battery_reader/get_battery_state",
      std::bind(
        &FakeBatteryReader::get_battery_state, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Publishing the BatteryState. Ready to get requets.!");
  }

  void set_battery_level(
    const std::shared_ptr<simple_interfaces::srv::SetBatteryLevel::Request> request,
    std::shared_ptr<simple_interfaces::srv::SetBatteryLevel::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Incoming request SetBatteryLevel");
    const std::lock_guard<std::mutex> lock(battery_state_mutex_);

    auto level_set = request->level;
    if (level_set > 1.0 || level_set < 0.0) {
      std::string error_message = "Battery Level must be between 0.0 and 1.0";
      response->is_ok = true;
      response->error_msg = error_message.c_str();

      RCLCPP_ERROR(this->get_logger(), error_message.c_str());
    } else {
      battery_state_.percentage = level_set;
      response->is_ok = true;
    }
  }

  void charge_battery(
    [[maybe_unused]] const std::shared_ptr<simple_interfaces::srv::ChargeBattery::Request> request,
    std::shared_ptr<simple_interfaces::srv::ChargeBattery::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Incoming request ChargeBattery");
    const std::lock_guard<std::mutex> lock(battery_state_mutex_);

    battery_state_.power_supply_status =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    response->is_ok = true;
  }

  void get_battery_state(
    [[maybe_unused]] const std::shared_ptr<simple_interfaces::srv::GetBatteryState::Request>
      request,
    std::shared_ptr<simple_interfaces::srv::GetBatteryState::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Incoming request GetBatteryState");
    const std::lock_guard<std::mutex> lock(battery_state_mutex_);

    response->state = battery_state_;
    response->is_ok = true;
  }

private:
  void timer_callback()
  {  // called every 500ms
    const std::lock_guard<std::mutex> lock(battery_state_mutex_);
    if (
      battery_state_.power_supply_status ==
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING) {
      battery_state_.percentage = std::min(battery_state_.percentage + 0.05, 1.0);  // recharing
    } else {
      battery_state_.percentage = std::max(battery_state_.percentage - 0.005, 0.0);  // discharging
    }

    if (battery_state_.percentage > 0.99) {  // it should never be above 1.0, but still
      battery_state_.power_supply_status =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
      RCLCPP_INFO(this->get_logger(), "Battery Full!");
    }
    battery_state_publisher_->publish(battery_state_);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher_;

  rclcpp::Service<simple_interfaces::srv::SetBatteryLevel>::SharedPtr srv_get_battery_level_;
  rclcpp::Service<simple_interfaces::srv::ChargeBattery>::SharedPtr srv_charge_battery_;
  rclcpp::Service<simple_interfaces::srv::GetBatteryState>::SharedPtr srv_get_battery_state_;
  sensor_msgs::msg::BatteryState battery_state_;
  std::mutex battery_state_mutex_;  // protects battery_state_, written in srv and sent to topic
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<FakeBatteryReader>();

  rclcpp::spin(node);
  rclcpp::shutdown();
}
