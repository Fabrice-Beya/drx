#include "DroneState.h"
#include "PIDController.h"
#include <stdio.h>
#include <time.h> // For clock() and CLOCKS_PER_SEC

int main()
{
    DroneState currentState = {0}; // Initialize all elements to 0

DroneState setpoint = {
    { // Explicit initialization for all elements of the state array
        5.0, // Target x position (meters)
        5.0, // Target y position (meters)
        10.0, // Target z position (meters) - Altitude
        0.0, // Target x velocity (m/s)
        0.0, // Target y velocity (m/s)
        0.0, // Target z velocity (m/s)
        0.0, // Target phi (roll) orientation (radians)
        0.0, // Target theta (pitch) orientation (radians)
        0.0, // Target psi (yaw) orientation (radians)
        0.0, // Target roll rate (radians/s)
        0.0, // Target pitch rate (radians/s)
        0.0  // Target yaw rate (radians/s)
    }
};

    // Assuming each array needs to have 12 elements for the 12 state variables
    double kp[12], ki[12], kd[12];

    // Initialize PID gain arrays with uniform values
    for (int i = 0; i < 12; i++) {
        kp[i] = 0.1;
        ki[i] = 0.01;
        kd[i] = 0.05;
    }
    PIDController controller;
    double controlActions[12];
    double dt = 0.1; // Time step for updates
    initializePIDController(&controller, kp, ki, kd);

    for (int i = 0; i < 50; i++)
    {
        clock_t start = clock();

        computeControlActions(&controller, &currentState, &setpoint, controlActions);
        // Apply control actions to update currentState here
        // This part is simplified; in real applications, dynamics calculations would update currentState

        clock_t end = clock();

        double time_taken = ((double)(end - start)) / (CLOCKS_PER_SEC * 0.000001); // in seconds
        printf("Control computation took %f microseconds.\n", time_taken);

        start = clock();
        applyControlActions(&currentState, controlActions, dt);
        end = clock();

        time_taken = ((double)(end - start)) / (CLOCKS_PER_SEC * 0.000001); // in seconds
        printf("Control actions update took %f microseconds.\n", time_taken);

        printDroneState(&currentState);
    }

    return 0;
}
