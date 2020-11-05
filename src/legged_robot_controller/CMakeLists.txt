cmake_minimum_required(VERSION 3.5)
project(legged_robot_controller)

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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(legged_robot_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories(
        include/${PROJECT_NAME}
        ${dynamixel_sdk_INCLUDE_DIRS}
)

add_library(control SHARED src/control.cpp)

ament_target_dependencies(control
        rclcpp
        tf2_ros
        std_srvs
        std_msgs
        geometry_msgs
        nav_msgs
        sensor_msgs
        legged_robot_msgs
        )

add_library(FABRIK2D SHARED src/FABRIK2D.cpp)
install(FILES include/legged_robot_controller/FABRIK2D.hpp DESTINATION include)

add_library(gait SHARED src/gait.cpp)
ament_target_dependencies(gait
        rclcpp
        geometry_msgs
        legged_robot_msgs
        )

add_library(ik SHARED src/ik.cpp)
ament_target_dependencies(ik
        rclcpp
        legged_robot_msgs
        )

add_library(servo_driver SHARED src/servo_driver.cpp)
ament_target_dependencies(servo_driver
        rclcpp
        sensor_msgs
        dynamixel_sdk
        )

add_executable(legged_robot_controller src/legged_robot_controller.cpp)
ament_target_dependencies(legged_robot_controller
        rclcpp
        std_srvs
        legged_robot_msgs
        )

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include/${PROJECT_NAME}
        )

install(
        TARGETS legged_robot_controller
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

#ament_export_targets(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(rclcpp geometry_msgs sensor_msgs std_msgs std_srvs nav_msgs legged_robot_msgs tf2_ros dynamixel_sdk)

ament_package()
