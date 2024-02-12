// src/DroneModel.cpp
#include "DroneModel.h"

// Constants for the drone's physical characteristics
const double gravity = 9.81;  // m/s^2
const double droneMass = 1.0; // kg
const double armLength = 0.25; // meters (distance from center to motor)
const double dragCoefficient = 0.1; // Simplified drag coefficient
const double momentOfInertia = 0.05; // kg.m^2, assuming uniform for roll, pitch, and yaw
const double dt = 0.1; // Time step for updates, in seconds

void DroneModel::applyControlActions(double forceX, double forceY, double thrust, double rollTorque, double pitchTorque, double yawTorque) {
    // Simplified physics calculations for demonstration
    state.vx += (forceX / droneMass) * dt;
    state.vy += (forceY / droneMass) * dt;
    state.vz += ((thrust / droneMass) - gravity) * dt;

    // Update positions based on velocities
    state.x += state.vx * dt;
    state.y += state.vy * dt;
    state.z += state.vz * dt;

    // Update orientation based on torques and moment of inertia (simplified)
    state.phi += (rollTorque / momentOfInertia) * dt;
    state.theta += (pitchTorque / momentOfInertia) * dt;
    state.psi += (yawTorque / momentOfInertia) * dt;
}


void DroneModel::updateState(const std::array<double, 4>& motorThrusts) {
    // Calculate total thrust and torques
    double totalThrust = 0.0;
    double torqueRoll = 0.0;
    double torquePitch = 0.0;
    double torqueYaw = 0.0; // Simplifying yaw control, not fully implemented here

    for (int i = 0; i < 4; ++i) {
        totalThrust += motorThrusts[i];
    }

    // Assuming a "+" configuration for simplicity
    torquePitch = (motorThrusts[0] - motorThrusts[2]) * armLength;
    torqueRoll = (motorThrusts[1] - motorThrusts[3]) * armLength;

    // Update translational dynamics
    double ax = (-dragCoefficient * state.vx) / droneMass;
    double ay = (-dragCoefficient * state.vy) / droneMass;
    // Assuming thrust acts along the z-axis and converting to acceleration
    double az = (totalThrust / droneMass) - gravity + (-dragCoefficient * state.vz) / droneMass;

    state.vx += ax * dt;
    state.vy += ay * dt;
    state.vz += az * dt;

    state.x += state.vx * dt;
    state.y += state.vy * dt;
    state.z += state.vz * dt;

    // Update rotational dynamics
    double alphaRoll = torqueRoll / momentOfInertia;
    double alphaPitch = torquePitch / momentOfInertia;
    // Simplified yaw dynamics (torqueYaw = 0 for this example)
    double alphaYaw = torqueYaw / momentOfInertia;

    state.p += alphaRoll * dt;
    state.q += alphaPitch * dt;
    state.r += alphaYaw * dt;

    state.phi += state.p * dt;
    state.theta += state.q * dt;
    state.psi += state.r * dt;
}
