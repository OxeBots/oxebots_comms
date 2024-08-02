cmake_minimum_required(VERSION 3.8)
project(oxebots_comms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Protobuf REQUIRED)
find_package(oxebots_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
  ${Protobuf_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_executable(
  sender_node
  src/sender_node.cpp
  src/robot_data_sender.cpp
  ${PROTO_SRCS}
)

ament_target_dependencies(sender_node rclcpp oxebots_interfaces)
target_link_libraries(sender_node ${Protobuf_LIBRARIES})

add_executable(
  game_receiver_node
  src/game_receiver_node.cpp
)

ament_target_dependencies(game_receiver_node rclcpp oxebots_interfaces Boost)
target_link_libraries(game_receiver_node ${Protobuf_LIBRARIES})

add_executable(
  geometry_receiver_node
  src/geometry_receiver_node.cpp
)

ament_target_dependencies(geometry_receiver_node rclcpp oxebots_interfaces Boost)
target_link_libraries(geometry_receiver_node ${Protobuf_LIBRARIES})

install(TARGETS
  sender_node
  game_receiver_node
  geometry_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
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
