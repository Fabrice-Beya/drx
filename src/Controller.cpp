#include "Controller.h"

void Controller::computeControlActions(const DroneState& currentState, const DroneState& setpoint, Eigen::VectorXd& controlActions) {
    Eigen::VectorXd currentVec = currentState.toVector();
    Eigen::VectorXd setpointVec = setpoint.toVector();
    Eigen::VectorXd errors = setpointVec - currentVec;

    // PID calculations
    integralErrors += errors * dt;
    Eigen::VectorXd derivative = (errors - previousErrors) / dt;
    controlActions = kp.array() * errors.array() + ki.array() * integralErrors.array() + kd.array() * derivative.array();

    previousErrors = errors;
}
