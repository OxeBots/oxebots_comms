cmake_minimum_required(VERSION 3.8)
project(oxebots_comms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Required packages
set(REQUIRED_PACKAGES
    ament_cmake
    oxebots_interfaces
    Protobuf
    rclcpp
)

set(REQUIRED_COMPONENTS
    Boost
)

foreach(pkg IN LISTS REQUIRED_PACKAGES)
    find_package(${pkg} REQUIRED)
endforeach()

foreach(pkg IN LISTS REQUIRED_COMPONENTS)
    find_package(${pkg} REQUIRED COMPONENTS system)
endforeach()

include_directories(
  include
)

# Macro for adding executables
macro(add_binary name)
    add_executable(${name}_node ${ARGN})
    ament_target_dependencies(${name}_node ${REQUIRED_PACKAGES} ${REQUIRED_COMPONENTS})
    target_link_libraries(${name}_node ${Protobuf_LIBRARIES})
endmacro()

add_binary(sender src/sender_node.cpp src/robot_data_sender.cpp)
add_binary(game_receiver src/game_receiver_node.cpp)
add_binary(gc_receiver src/gc_receiver_node.cpp)

# Install targets
install(TARGETS
    sender_node
    game_receiver_node
    gc_receiver_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
