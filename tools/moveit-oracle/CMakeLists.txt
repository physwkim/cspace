cmake_minimum_required(VERSION 3.22)
project(moveit_oracle CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(moveit_core REQUIRED)
find_package(urdfdom REQUIRED)
find_package(srdfdom REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(random_numbers REQUIRED)

add_executable(moveit_oracle src/oracle.cpp)

# Namespaced targets, not bare library names: moveit_core exports
# moveit_core::moveit_robot_model, and a bare `moveit_robot_model` is treated by
# CMake as a raw -l flag, which links but carries no include directories -- the
# build then fails on a missing moveit/robot_model/robot_model.hpp.
target_link_libraries(moveit_oracle
  PRIVATE
    moveit_core::moveit_robot_model
    moveit_core::moveit_robot_state
    moveit_core::moveit_collision_detection
    urdfdom::urdf_parser
    srdfdom::srdfdom
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    ${random_numbers_LIBRARIES}
)
target_include_directories(moveit_oracle PRIVATE ${random_numbers_INCLUDE_DIRS})

install(TARGETS moveit_oracle RUNTIME DESTINATION bin)
