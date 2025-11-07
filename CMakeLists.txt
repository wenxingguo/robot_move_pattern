cmake_minimum_required(VERSION 3.16)
project(ROBOR_MOVE_PATTERN)

find_package(ur_rtde REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)

add_executable(ROBOR_MOVE_PATTERN main.cpp robot_move_pattern.cpp)
target_link_libraries(ROBOR_MOVE_PATTERN ur_rtde::rtde Eigen3::Eigen)

pybind11_add_module(robot_move_pattern robot_move_pattern_python/robot_move_pattern_py.cpp robot_move_pattern.cpp)
target_link_libraries(robot_move_pattern PRIVATE Eigen3::Eigen)

