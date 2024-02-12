// include/DroneState.h
#pragma once
#include <iostream>
#include <Eigen/Dense>

class DroneState {
public:
    double x = 0.0, y = 0.0, z = 0.0, phi = 0.0, theta = 0.0, psi = 0.0;

    // Convert DroneState to Eigen::VectorXd
    Eigen::VectorXd toVector() const {
        Eigen::VectorXd stateVec(6);
        stateVec << x, y, z, phi, theta, psi;
        return stateVec;
    }

    // Update DroneState from Eigen::VectorXd
    void fromVector(const Eigen::VectorXd& stateVec) {
        x = stateVec(0);
        y = stateVec(1);
        z = stateVec(2);
        phi = stateVec(3);
        theta = stateVec(4);
        psi = stateVec(5);
    }
};

