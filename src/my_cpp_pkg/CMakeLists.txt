cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)

add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

add_executable(robot_news_station_cpp src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station_cpp rclcpp example_interfaces)

add_executable(smartphone_cpp src/smartphone.cpp)
ament_target_dependencies(smartphone_cpp rclcpp example_interfaces)

add_executable(number_publisher src/activity_002_number_pub.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces)

add_executable(number_pub_and_sub src/activity_002_pub_and_sub.cpp)
ament_target_dependencies(number_pub_and_sub rclcpp example_interfaces)

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

add_executable(add_two_ints_client_no_oop src/add_two_ints_client_no_oop.cpp)
ament_target_dependencies(add_two_ints_client_no_oop rclcpp example_interfaces)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)

add_executable(hardware_status_publisher src/hardware_status_publisher.cpp)
ament_target_dependencies(hardware_status_publisher rclcpp my_robot_interfaces)

add_executable(compute_rectangle_area_server src/compute_rectangle_area_server.cpp)
ament_target_dependencies(compute_rectangle_area_server rclcpp my_robot_interfaces)

# activity 004
add_executable(battery_node src/battery_node.cpp)
ament_target_dependencies(battery_node rclcpp my_robot_interfaces)

add_executable(led_panel_node src/led_panel_node.cpp)
ament_target_dependencies(led_panel_node rclcpp my_robot_interfaces)


install(TARGETS 
  cpp_node
  robot_news_station_cpp
  smartphone_cpp
  number_publisher
  number_pub_and_sub
  add_two_ints_server
  add_two_ints_client_no_oop
  add_two_ints_client
  hardware_status_publisher
  compute_rectangle_area_server
  battery_node
  led_panel_node
  DESTINATION lib/${PROJECT_NAME})
ament_package()
