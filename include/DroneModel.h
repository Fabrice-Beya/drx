// include/DroneModel.h
#pragma once
#include "DroneState.h"
#include <array>

class DroneModel {
public:
    DroneState state;
    void updateState(const std::array<double, 4>& motorThrusts);
    void applyControlActions(double forceX, double forceY, double thrust, double rollTorque, double pitchTorque, double yawTorque);
};

