cmake_minimum_required(VERSION 3.5)
project(legged_robot_bringup)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(legged_robot_parameter_server src/legged_robot_parameter_server.cpp)
ament_target_dependencies(legged_robot_parameter_server rclcpp)

install(TARGETS
        legged_robot_parameter_server
        DESTINATION lib/${PROJECT_NAME}
        )
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
        )
install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
        )

ament_package()
