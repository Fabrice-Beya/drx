cmake_minimum_required(VERSION 3.10) # Specify the minimum version of CMake required

project(DroneControlSystem C) # Name your project and specify it's a C project

# Set the C standard to use
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED True)

# Add executable target with source files
add_executable(DroneControlSystem src/main.c src/DroneState.c src/PIDController.c)

# If you have any include directories, specify them here
include_directories(include)

# If you need to link libraries, specify them here
#target_link_libraries(DroneControlSystem m) # Link math library if needed
