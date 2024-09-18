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
    geometry_msgs
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

# list that associates the node name with the source files
set(NODES
    sender_node "src/sender_node.cpp src/robot_data_sender.cpp"
    game_receiver_node "src/game_receiver_node.cpp"
    gc_receiver_node "src/gc_receiver_node.cpp"
)

# Macro for adding executable nodes
macro(add_node name)
    add_executable(${name} ${ARGN})
    ament_target_dependencies(${name} ${REQUIRED_PACKAGES} ${REQUIRED_COMPONENTS})
    target_link_libraries(${name} ${Protobuf_LIBRARIES})
    install(TARGETS ${name} DESTINATION lib/${PROJECT_NAME})
endmacro()

list(LENGTH NODES nodes_length)
math(EXPR last_index "${nodes_length} - 2")

foreach(i RANGE 0 ${last_index} 2)
    list(GET NODES ${i} node_name)
    math(EXPR i++ "${i} + 1")
    list(GET NODES ${i++} node_srcs)
    string(REPLACE " " ";" node_srcs ${node_srcs})
    add_node(${node_name} ${node_srcs})
endforeach()

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
