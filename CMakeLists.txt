cmake_minimum_required(VERSION 3.8)
project(cpp_topics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

add_executable(new src/virtual_bot.cpp)
add_executable(square src/square.cpp)
add_executable(rotate src/rotate.cpp)
add_executable(qr_code src/qr.cpp)

ament_target_dependencies(new rclcpp geometry_msgs tf2 tf2_ros std_msgs visualization_msgs nav_msgs)
ament_target_dependencies(square rclcpp geometry_msgs std_msgs nav_msgs tf2)
ament_target_dependencies(rotate rclcpp geometry_msgs std_msgs nav_msgs tf2)
ament_target_dependencies(qr_code rclcpp std_msgs)

install(TARGETS
  new
  square
  qr_code
  rotate
  DESTINATION lib/${PROJECT_NAME})

ament_package()
