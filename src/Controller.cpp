#include "Controller.h"

// Helper function for PID control
double computePID(double setpoint, double current, double& integralError, double& previousError, const Controller::PIDGains& gains, double dt) {
    double error = setpoint - current;
    integralError += error * dt;
    double derivative = (error - previousError) / dt;
    double output = gains.kp * error + gains.ki * integralError + gains.kd * derivative;
    previousError = error;
    return output;
}

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
