//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_CONTOURING_ERROR_H
#define MPCC_CONTOURING_ERROR_H

#include <Eigen/Dense>
#include "../models/state.h"
#include "../splines/cubic_spline2d.h"
#include "cost_types.h"

using Eigen::Matrix;

// Position and partial derivatives of x, y and yaw with respect to theta (spline parameter)
struct RefPoint {
    const double x;
    const double y;
    const double dx;
    const double dy;
    const double yaw;
    const double dyaw;
};

// Contains the exact non-linear error and Jacobian of the error at the predicted "xk"
struct ContouringError {
    const Matrix<double, 2, 1> error;
    const Matrix<double, 2, NX> d_error;
};

class ContouringErrorDelegate {
public:
    static RefPoint getRefPoint(const CubicSpline2D &path, const State &xk);
    static ContouringError getContouringError(const CubicSpline2D &path, const State &xk);
};

#endif //MPCC_CONTOURING_ERROR_H
