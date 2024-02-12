#ifndef DRONESTATE_H
#define DRONESTATE_H

#include <stdio.h>

// Represents the drone's state: position, velocity, orientation, and angular velocity
typedef struct {
    double state[12]; // x, y, z, vx, vy, vz, phi, theta, psi, p, q, r
} DroneState;

void printDroneState(const DroneState* state);
void applyControlActions(DroneState* state, const double controlActions[12], double dt);

#endif // DRONESTATE_H
