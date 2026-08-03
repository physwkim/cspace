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

add_executable(moveit_oracle src/oracle.cpp)
target_link_libraries(moveit_oracle
  PRIVATE
    moveit_robot_model
    moveit_robot_state
    ${urdfdom_LIBRARIES}
    srdfdom::srdfdom
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
)
target_include_directories(moveit_oracle PRIVATE ${urdfdom_INCLUDE_DIRS})

install(TARGETS moveit_oracle RUNTIME DESTINATION bin)
