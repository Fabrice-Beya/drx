#include "DroneModel.h"
#include "Controller.h"
#include <Eigen/Dense>
#include <iostream>
#include <chrono>

int main() {
    // Define PID gains for x, y, z, roll, pitch, yaw
   Eigen::VectorXd kp(12), ki(12), kd(12);
    kp << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1; // Placeholder gains for all 12 states
    ki << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    kd << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05;

    double dt = 0.1; // Time step

    Controller controller(kp, ki, kd, dt);

    DroneModel drone;
    DroneState setpoint;
    // Define a comprehensive setpoint with non-zero values
Eigen::VectorXd setpointVector(12);
setpointVector << 
    5.0,  // Target x position (meters)
    5.0,  // Target y position (meters)
    10.0, // Target z position (meters) - Altitude of 10 meters
    0.0,  // Target x velocity (m/s) - Stationary in x
    0.0,  // Target y velocity (m/s) - Stationary in y
    0.0,  // Target z velocity (m/s) - Stationary in z
    0.0,  // Target roll angle (radians) - Level
    0.0,  // Target pitch angle (radians) - Level
    0.0,  // Target yaw angle (radians) - No specific orientation
    0.0,  // Target roll rate (radians/s) - No rotation
    0.0,  // Target pitch rate (radians/s) - No rotation
    0.0;  // Target yaw rate (radians/s) - No rotation

    setpoint.fromVector(setpointVector);
    for (int i = 0; i < 50; ++i) { // Simulate 50 control loop iterations
        Eigen::VectorXd controlActions = Eigen::VectorXd::Zero(12); // Placeholder for control actions calculated by the controller

        // Timing computeControlActions
        auto startCompute = std::chrono::high_resolution_clock::now();
        controller.computeControlActions(drone.state, setpoint, controlActions);
        auto stopCompute = std::chrono::high_resolution_clock::now();

        // Timing applyControlActions
        auto startApply = std::chrono::high_resolution_clock::now();
        drone.applyControlActions(controlActions); // Apply the computed control actions to the drone
        auto stopApply = std::chrono::high_resolution_clock::now();

        // Calculate and display the durations
        auto durationCompute = std::chrono::duration_cast<std::chrono::microseconds>(stopCompute - startCompute);
        auto durationApply = std::chrono::duration_cast<std::chrono::microseconds>(stopApply - startApply);
        std::cout << "Iteration " << i+1 << ":\n"
                  << "computeControlActions took " << durationCompute.count() << " microseconds.\n"
                  << "applyControlActions took " << durationApply.count() << " microseconds.\n";

        // Optionally, print the updated state to observe changes
        drone.state.print();
    }

    return 0;
}
