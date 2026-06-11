cmake_minimum_required(VERSION 3.8)
project(bumperbot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------
# ROS2 dependencies
# -------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# -------------------------
# LibSerial via pkg-config
# -------------------------
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# -------------------------
# Python package install
# -------------------------
ament_python_install_package(${PROJECT_NAME})

# -------------------------
# Main hardware library
# -------------------------
add_library(${PROJECT_NAME} SHARED
  src/bumperbot_interface.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${LIBSERIAL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${LIBSERIAL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
  hardware_interface
  bumperbot_interface.xml
)

# -------------------------
# Serial receiver node
# -------------------------
add_executable(simple_serial_receiver
  src/simple_serial_receiver.cpp
)

ament_target_dependencies(simple_serial_receiver
  rclcpp
  std_msgs
)

target_include_directories(simple_serial_receiver PUBLIC
  ${LIBSERIAL_INCLUDE_DIRS}
)

target_link_libraries(simple_serial_receiver
  ${LIBSERIAL_LIBRARIES}
)

# -------------------------
# Serial transmitter node
# -------------------------
add_executable(simple_serial_transmitter
  src/simple_serial_transmitter.cpp
)

ament_target_dependencies(simple_serial_transmitter
  rclcpp
  std_msgs
)

target_include_directories(simple_serial_transmitter PUBLIC
  ${LIBSERIAL_INCLUDE_DIRS}
)

target_link_libraries(simple_serial_transmitter
  ${LIBSERIAL_LIBRARIES}
)

# -------------------------
# Install targets
# -------------------------
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    simple_serial_receiver
    simple_serial_transmitter
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/simple_serial_transmitter.py
  ${PROJECT_NAME}/simple_serial_receiver.py
  ${PROJECT_NAME}/mpu6050_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------
# Export
# -------------------------
ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()