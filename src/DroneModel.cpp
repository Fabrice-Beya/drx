#include "DroneModel.h"

void DroneModel::applyControlActions(const Eigen::VectorXd& controlActions) {
    if (controlActions.size() != 12) {
        std::cerr << "Error: Expected controlActions vector of size 12." << std::endl;
        return;
    }
    // Assuming control actions directly influence state for simplicity
    state.fromVector(state.toVector() + controlActions);
}