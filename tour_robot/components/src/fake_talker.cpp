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

#include <rclcpp/rclcpp.hpp>
#include <tour_interfaces/srv/say_text.hpp>

std::shared_ptr<rclcpp::Node> node;

void say_text(
  const std::shared_ptr<tour_interfaces::srv::SayText::Request> request,
  std::shared_ptr<tour_interfaces::srv::SayText::Response> response)
{
  RCLCPP_INFO(node->get_logger(), "Incoming request say_text");

  RCLCPP_INFO(node->get_logger(), "--------------------");
  RCLCPP_INFO(node->get_logger(), "Saying: %s", request->text.c_str());
  RCLCPP_INFO(node->get_logger(), "--------------------");

  response->is_ok = true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("fake_talker_server");
  auto say_text_srv = node->create_service<tour_interfaces::srv::SayText>("say_text", &say_text);

  RCLCPP_INFO(rclcpp::get_logger("say_text_srv"), "Ready to get requests.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
