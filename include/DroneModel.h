#ifndef DRONEMODEL_H
#define DRONEMODEL_H

#include <Eigen/Dense>
#include <iostream>
#include "DroneState.h"


class DroneModel {
public:
    DroneState state; // Represents the current state of the drone

    // Applies control actions to the drone's state
    void applyControlActions(const Eigen::VectorXd& controlActions);

    // Additional methods might be added here for further functionality
};

#endif // DRONEMODEL_H
