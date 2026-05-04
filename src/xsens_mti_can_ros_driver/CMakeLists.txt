cmake_minimum_required(VERSION 3.8)
project(xsens_mti_can_ros_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/XsStatusWord.msg"
  DEPENDENCIES std_msgs
)

add_executable(xsens_mti_can_ros_node
  src/main.cpp
  src/xsens_parser.cpp
  src/xscaninterface.cpp
  src/xsens_time_handler.cpp
)

target_include_directories(xsens_mti_can_ros_node PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/src
)

ament_target_dependencies(xsens_mti_can_ros_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  diagnostic_msgs
  diagnostic_updater
  tf2
  tf2_ros
)

# Link the generated message typesupport so headers like
# xsens_mti_can_ros_driver/msg/xs_status_word.hpp resolve at link time.
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(xsens_mti_can_ros_node "${cpp_typesupport_target}")

install(TARGETS xsens_mti_can_ros_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch param config rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
