#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "DroneState.h"

typedef struct {
    double kp[12], ki[12], kd[12]; // PID gains for each state variable
    double integralError[12]; // Integral of error for each state variable
    double previousError[12]; // Previous error for derivative calculation
} PIDController;

void initializePIDController(PIDController* controller, const double kp[], const double ki[], const double kd[]);
void computeControlActions(PIDController* controller, const DroneState* currentState, const DroneState* setpoint, double controlActions[12]);

#endif // PIDCONTROLLER_H
