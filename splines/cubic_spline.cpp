//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "cubic_spline.h"

CubicSpline::CubicSpline(const VectorXd &t_data, const VectorXd &ft_data) {
    t_spline = t_data;
    ft_spline = ft_data;
    setSplineCoefficients(t_data, ft_data);
}

int CubicSpline::searchIndex(const double t) const {
    return mpcc::binary_search_left(t_spline, t);
}

double CubicSpline::getPosition(double t) const {
    t = constrainInput(t);
    int index = searchIndex(t);
    double h = t - t_spline(index);
    double h2 = h * h;
    double h3 = h2 * h;
    return a(index) + b(index)*h + c(index)*h2 + d(index)*h3;
}

double CubicSpline::getDerivative(double t) const {
    t = constrainInput(t);
    int index = searchIndex(t);
    double h = t - t_spline(index);
    double h2 = h * h;
    return b(index) + 2*c(index)*h + 3*d(index)*h2;
}

double CubicSpline::getSecondDerivative(double t) const {
    t = constrainInput(t);
    int index = searchIndex(t);
    double h = t - t_spline(index);
    return c(index)*h + 6*d(index)*h;
}

void CubicSpline::setSplineCoefficients(const VectorXd &t_data, const VectorXd &ft_data) {
    const int n_points = t_data.size();
    int i;

    // Algorithm can be found on
    // https://fac.ksu.edu.sa/sites/default/files/numerical_analysis_9th.pdf#page=168
    a.setZero(n_points);
    b.setZero(n_points - 1);
    c.setZero(n_points);
    d.setZero(n_points - 1);

    // Helper variables to compute a, b, c and d
    VectorXd mu = VectorXd::Zero(n_points - 1);
    VectorXd h = VectorXd::Zero(n_points - 1);
    VectorXd alpha = VectorXd::Zero(n_points - 1);
    VectorXd l = VectorXd::Zero(n_points);
    VectorXd z = VectorXd::Zero(n_points);

    // "a" is equal to y
    a = ft_data;

    // Compute intervals of t_data, called h
    for (i = 0; i < n_points-1; i++) {
        h(i) = t_data(i+1) - t_data(i);
    }

    // Compute alpha
    for(i = 1; i < n_points-1; i++) {
        alpha(i) = 3.0/h(i)*(a(i+1)-a(i)) - 3.0/h(i-1)*(a(i)-a(i-1));
    }

    for (i = 1; i < n_points - 1; i++) {
        l(i) = 2.0 * (t_data(i+1) - t_data(i-1)) - h(i-1)*mu(i-1);
        mu(i) = h(i) / l(i);
        z(i) = (alpha(i) - h(i-1)*z(i-1)) / l(i);
    }

    l(n_points-1) = 1.0;
    z(n_points-1) = 0.0;
    // "c" corresponds to 2nd derivative of the spline. At the end-point, we want the 2nd derivative to be 0
    c(n_points - 1) = 0.0;

    // Now we work through backwards to compute b, c, d
    for (i = n_points - 2; i >= 0; i--) {
        c(i) = z(i) - mu(i) * c(i + 1);
        b(i) = (a(i + 1) - a(i)) / h(i) -
               (h(i)*(c(i + 1) + 2.0 * c(i))) / 3.0;
        d(i) = (c(i + 1) - c(i)) / (3.0 * h(i));
    }
}

double CubicSpline::constrainInput(double t) const {
    int n_points = t_spline.size();
    // If t is outside the minimum of the spline parameter, set it to the minimum
    if (t < t_spline(0)) {
        return t_spline(0);
    }
    // If t is outside the maximum of the spline parameter, set it to the maximum
    if (t > t_spline(n_points-1)) {
        return t_spline(n_points-1);
    }
    return t;
}
