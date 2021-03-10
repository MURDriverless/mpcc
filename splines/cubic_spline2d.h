//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_CUBIC_SPLINE2D_H
#define MPCC_CUBIC_SPLINE2D_H

#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "../models/state.h"
#include "cubic_spline.h"

using Eigen::VectorXd;
using Eigen::Vector2d;

class CubicSpline2D {
public:
    CubicSpline2D();
    CubicSpline2D(const VectorXd &x_data, const VectorXd &y_data, double max_dist_proj_params);
    double getLength() const;
    Vector2d getPosition(double s) const;
    Vector2d getDerivative(double s) const;
    Vector2d getSecondDerivative(double s) const;
    double projectOnSpline(const State &xk) const;
private:
    // Function to calculate Pythagorean distance between successive data-points
    static VectorXd calcLineDistances(const VectorXd &x_data, const VectorXd &y_data);
    static VectorXd cumulativeSum(const VectorXd &xy_data);
    double constrainInput(double s) const;
    VectorXd s_vector;
    CubicSpline sx;
    CubicSpline sy;
    double max_dist_proj;
};

#endif //MPCC_CUBIC_SPLINE2D_H
