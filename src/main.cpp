#include "DroneModel.h"
#include "Controller.h"
#include <Eigen/Dense>
#include <iostream>
#include <chrono>

int main() {
    Eigen::VectorXd kp(6), ki(6), kd(6);
    kp << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    ki << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    kd << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05;
    double dt = 0.1; // Time step

    Controller controller(kp, ki, kd, dt);

    DroneState droneState, setpoint;
    setpoint.fromVector(Eigen::VectorXd::Zero(6)); // Example target state
    setpoint.z = 10.0; // Target altitude

    for (int i = 0; i < 50; ++i) {
        Eigen::VectorXd controlActions(6);

        // Start timing for computeControlActions
        auto startCompute = std::chrono::high_resolution_clock::now();
        
        controller.computeControlActions(droneState, setpoint, controlActions);

        // Stop timing and calculate duration
        auto stopCompute = std::chrono::high_resolution_clock::now();
        auto durationCompute = std::chrono::duration_cast<std::chrono::microseconds>(stopCompute - startCompute).count();

        // Start timing for state update (assuming this is handled within fromVector for simplicity)
        auto startUpdate = std::chrono::high_resolution_clock::now();

        droneState.fromVector(droneState.toVector() + controlActions * dt); // Simplified state update logic

        // Stop timing and calculate duration
        auto stopUpdate = std::chrono::high_resolution_clock::now();
        auto durationUpdate = std::chrono::duration_cast<std::chrono::microseconds>(stopUpdate - startUpdate).count();

        // Print the timings
        std::cout << "Iteration " << i+1 << ":\n"
                  << "computeControlActions took " << durationCompute << " microseconds.\n"
                  << "State update took " << durationUpdate << " microseconds.\n\n";
    }

    return 0;
}
