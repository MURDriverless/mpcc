//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_CUBIC_SPLINE_H
#define MPCC_CUBIC_SPLINE_H

#include <iostream>
#include <Eigen/Dense>
#include "../utils/binary_search.h"

using Eigen::VectorXd;

class CubicSpline {
public:
    CubicSpline() = default;
    CubicSpline(const VectorXd &t_data, const VectorXd &ft_data);
    int searchIndex(double t) const;
    double getPosition(double t) const;
    double getDerivative(double t) const;
    double getSecondDerivative(double t) const;
    // Spline parameter breakpoints
    VectorXd t_spline;
    VectorXd ft_spline; // f(t)
    // Spline coefficients
    VectorXd a;
    VectorXd b;
    VectorXd c;
    VectorXd d;
private:
    void setSplineCoefficients(const VectorXd &t_data, const VectorXd &ft_data);
    double constrainInput(double t) const;
};

#endif //MPCC_CUBIC_SPLINE_H
