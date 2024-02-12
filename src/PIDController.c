#include "PIDController.h"
#include <string.h> // For memcpy

void initializePIDController(PIDController* controller, const double kp[], const double ki[], const double kd[]) {
    memcpy(controller->kp, kp, sizeof(controller->kp));
    memcpy(controller->ki, ki, sizeof(controller->ki));
    memcpy(controller->kd, kd, sizeof(controller->kd));
    memset(controller->integralError, 0, sizeof(controller->integralError));
    memset(controller->previousError, 0, sizeof(controller->previousError));
}

void computeControlActions(PIDController* controller, const DroneState* currentState, const DroneState* setpoint, double controlActions[12]) {
    for (int i = 0; i < 12; i++) {
        double error = setpoint->state[i] - currentState->state[i];
        controller->integralError[i] += error;
        double derivative = error - controller->previousError[i];
        controlActions[i] = controller->kp[i] * error + controller->ki[i] * controller->integralError[i] + controller->kd[i] * derivative;
        controller->previousError[i] = error;
    }
}
