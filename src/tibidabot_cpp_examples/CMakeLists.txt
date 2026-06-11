cmake_minimum_required(VERSION 3.8)
project(tibidabot_cpp_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(bumperbot_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs)

add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber rclcpp std_msgs)

add_executable(simple_lifecycle_node src/simple_lifecycle_node.cpp)
ament_target_dependencies(simple_lifecycle_node rclcpp rclcpp_lifecycle std_msgs)

#block of instruction to compile action server
add_library(simple_action_server SHARED src/simple_action_server.cpp)
target_include_directories(simple_action_server PRIVATE
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>)
target_compile_definitions(simple_action_server
 PRIVATE "SIMPLE_ACTION_SERVER_CPP_BUILDING_DLL")
ament_target_dependencies(simple_action_server
"bumperbot_msgs"
 "rclcpp"
 "rclcpp_action"
 "rclcpp_components")
rclcpp_components_register_node(simple_action_server 
 PLUGIN "tibidabot_cpp_examples::SimpleActionServer" 
 EXECUTABLE simple_action_server_node
)
#end of compilation istruction server block

#block of instruction to compile action client
add_library(simple_action_client SHARED src/simple_action_client.cpp)
target_include_directories(simple_action_client PRIVATE
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>)
target_compile_definitions(simple_action_client
 PRIVATE "SIMPLE_ACTION_CLIENT_CPP_BUILDING_DLL")
ament_target_dependencies(simple_action_client
"bumperbot_msgs"
 "rclcpp"
 "rclcpp_action"
 "rclcpp_components")
rclcpp_components_register_node(simple_action_client 
 PLUGIN "tibidabot_cpp_examples::SimpleActionClient" 
 EXECUTABLE simple_action_client_node
)
#end of compilation istruction client block



install(TARGETS
      simple_publisher
      simple_subscriber
      simple_lifecycle_node
      DESTINATION 
      lib/${PROJECT_NAME})     

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  simple_action_server
  simple_action_client
  ARCHIVE DESTINATION lib 
  LIBRARY DESTINATION lib 
  RUNTIME DESTINATION lib 
  )

ament_package()
