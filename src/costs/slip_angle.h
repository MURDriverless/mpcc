//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_SLIP_ANGLE_H
#define MPCC_SLIP_ANGLE_H

#include <Eigen/Dense>
#include "../models/state.h"

using Eigen::Matrix;

// General container to hold linearisation variables: setpoint value (bar) and Jacobian (jac)
struct LinearisedVar {
    const double bar;
    const Matrix<double, 1, NX> jac;
};

class SlipAngleDelegate {
public:
    // Functions to determine vehicular slip angle beta, keep in mind that alpha denotes tire slip angle
    static LinearisedVar getBetaKin(const State &xk, double lf, double lr);
    static LinearisedVar getBetaDyn(const State &xk);
};

#endif //MPCC_SLIP_ANGLE_H
