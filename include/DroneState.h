// include/DroneState.h
#pragma once
#include <iostream>
#include <Eigen/Dense>

class DroneState {
public:
    double x = 0.0, y = 0.0, z = 0.0;
    double vx = 0.0, vy = 0.0, vz = 0.0;
    double phi = 0.0, theta = 0.0, psi = 0.0;
    double p = 0.0, q = 0.0, r = 0.0;

    Eigen::VectorXd toVector() const {
        Eigen::VectorXd vec(12);
        vec << x, y, z, vx, vy, vz, phi, theta, psi, p, q, r;
        return vec;
    }

    void fromVector(const Eigen::VectorXd& vec) {
        if (vec.size() != 12) throw std::runtime_error("Vector size mismatch");
        x = vec(0); y = vec(1); z = vec(2);
        vx = vec(3); vy = vec(4); vz = vec(5);
        phi = vec(6); theta = vec(7); psi = vec(8);
        p = vec(9); q = vec(10); r = vec(11);
    }

    void print() const {
        std::cout << "State: Pos(" << x << ", " << y << ", " << z 
                  << ") Vel(" << vx << ", " << vy << ", " << vz 
                  << ") Orient(" << phi << ", " << theta << ", " << psi 
                  << ") AngVel(" << p << ", " << q << ", " << r << ")\n";
    }
};

