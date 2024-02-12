#include "DroneState.h"

void printDroneState(const DroneState* state) {
    printf("Position: [%.2f, %.2f, %.2f], Velocity: [%.2f, %.2f, %.2f], Orientation: [%.2f, %.2f, %.2f], Angular Velocity: [%.2f, %.2f, %.2f]\n",
           state->state[0], state->state[1], state->state[2],
           state->state[3], state->state[4], state->state[5],
           state->state[6], state->state[7], state->state[8],
           state->state[9], state->state[10], state->state[11]);
}

void applyControlActions(DroneState* state, const double controlActions[12], double dt) {
    // Directly apply control actions to update state.
    // Assuming control actions for positions (x, y, z) and orientations (phi, theta, psi)
    // are direct adjustments rather than velocities.

    for (int i = 0; i < 3; i++) {
        // Directly update positions and orientations based on control actions
        state->state[i] += controlActions[i] * dt; // Update x, y, z positions
        state->state[6 + i] += controlActions[6 + i] * dt; // Update phi, theta, psi orientations
    }

    // Update velocities and angular velocities based on control actions.
    // This assumes that part of the control actions directly set the velocities and angular velocities.
    state->state[3] = controlActions[3]; // Set vx
    state->state[4] = controlActions[4]; // Set vy
    state->state[5] = controlActions[5]; // Set vz
    state->state[9] = controlActions[9]; // Set p (roll rate)
    state->state[10] = controlActions[10]; // Set q (pitch rate)
    state->state[11] = controlActions[11]; // Set r (yaw rate)
}
