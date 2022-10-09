cmake_minimum_required(VERSION 3.8)
project(tour_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tour_interfaces REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_executable(fake_battery_reader
  src/fake_battery_reader.cpp
)

ament_target_dependencies(fake_battery_reader
 rclcpp
 tour_interfaces
 sensor_msgs 
)

install (TARGETS
  fake_battery_reader
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(fake_talker
  src/fake_talker.cpp
)

ament_target_dependencies(fake_talker
 rclcpp
 tour_interfaces
)

install (TARGETS
  fake_talker
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(simple_tour_scheduler
  src/simple_tour_scheduler.cpp
)

ament_target_dependencies(simple_tour_scheduler
 rclcpp
 tour_interfaces
 yaml_cpp_vendor
 )

install (TARGETS
simple_tour_scheduler
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
