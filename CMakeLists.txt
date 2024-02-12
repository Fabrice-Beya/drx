# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(DroneSimulation)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(include)

# Add executable
add_executable(DroneSimulation src/main.cpp src/DroneModel.cpp src/Controller.cpp)

target_include_directories(DroneSimulation PRIVATE include)

target_link_libraries(DroneSimulation Eigen3::Eigen)
