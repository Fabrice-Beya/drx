// include/Controller.h
#pragma once
#include "DroneState.h"
#include <Eigen/Dense>

class Controller {
public:
    Eigen::VectorXd kp, ki, kd; // PID gains as vectors
    Eigen::VectorXd integralErrors, previousErrors; // Error vectors
    double dt; // Time step

    Controller(const Eigen::VectorXd& kp, const Eigen::VectorXd& ki, const Eigen::VectorXd& kd, double dtValue)
        : kp(kp), ki(ki), kd(kd), dt(dtValue) {
        integralErrors = Eigen::VectorXd::Zero(12);
        previousErrors = Eigen::VectorXd::Zero(12);
    }

    void computeControlActions(const DroneState& currentState, const DroneState& setpoint, Eigen::VectorXd& controlActions);
};

