#===============================================================================
# CMakeLists.txt - Kinematics-Aware Navigation Package (Production)
#===============================================================================
cmake_minimum_required(VERSION 3.14)
project(kinematic_nav_pkg VERSION 1.0.0 LANGUAGES CXX)

#-------------------------------------------------------------------------------
# Compiler Configuration
#-------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3 -DNDEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-O0 -g -DDEBUG)
endif()

#-------------------------------------------------------------------------------
# Find Dependencies
#-------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

#-------------------------------------------------------------------------------
# Include Directories
#-------------------------------------------------------------------------------
include_directories(include)

#-------------------------------------------------------------------------------
# Executables
#-------------------------------------------------------------------------------
add_executable(point_to_point_navigator
    src/point_to_point_navigator.cpp
)

ament_target_dependencies(point_to_point_navigator
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

#-------------------------------------------------------------------------------
# Installation
#-------------------------------------------------------------------------------
install(TARGETS
    point_to_point_navigator
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
    OPTIONAL
)

install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
    OPTIONAL
)

#-------------------------------------------------------------------------------
# Export Package Information
#-------------------------------------------------------------------------------
ament_export_include_directories(include)
ament_export_dependencies(
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
)

#-------------------------------------------------------------------------------
# Testing
#-------------------------------------------------------------------------------
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)

    ament_lint_auto_find_test_dependencies()

    # PID Controller Tests
    ament_add_gtest(test_pid_controller test/test_pid_controller.cpp)
    target_include_directories(test_pid_controller PRIVATE include)

    # Obstacle Detector Tests
    ament_add_gtest(test_obstacle_detector test/test_obstacle_detector.cpp)
    target_include_directories(test_obstacle_detector PRIVATE include)

    # Velocity Smoother Tests
    ament_add_gtest(test_velocity_smoother test/test_velocity_smoother.cpp)
    target_include_directories(test_velocity_smoother PRIVATE include)

    # Recovery Manager Tests
    ament_add_gtest(test_recovery_manager test/test_recovery_manager.cpp)
    target_include_directories(test_recovery_manager PRIVATE include)
endif()

#-------------------------------------------------------------------------------
# Package Registration
#-------------------------------------------------------------------------------
ament_package()
