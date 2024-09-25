
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
    sender_node "src/sender.cpp src/robot_data_sender.cpp"
    gc_receiver_node "src/gc_receiver.cpp"
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

add_executable(comms_composition src/composition.cpp src/game_receiver.cpp)
ament_target_dependencies(comms_composition ${REQUIRED_PACKAGES} ${REQUIRED_COMPONENTS})
target_link_libraries(comms_composition ${Protobuf_LIBRARIES})
install(TARGETS comms_composition DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY include/
    DESTINATION include
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    set(ament_cmake_lint_FOUND TRUE)

    # Fix the gtest target to include the correct file
    ament_add_gtest(${PROJECT_NAME}_game_receiver test/game_receiver_test.cpp src/game_receiver.cpp)
    target_include_directories(${PROJECT_NAME}_game_receiver PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    ament_target_dependencies(${PROJECT_NAME}_game_receiver ${REQUIRED_PACKAGES} ${REQUIRED_COMPONENTS})
    target_link_libraries(${PROJECT_NAME}_game_receiver ${Protobuf_LIBRARIES})
    
endif()

ament_package()

# Set CMP0115 policy to avoid warnings
cmake_policy(SET CMP0115 NEW)

