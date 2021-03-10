//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "cubic_spline2d.h"

using Eigen::ArrayXd;
using std::vector;

CubicSpline2D::CubicSpline2D() { max_dist_proj = 3.0; };

CubicSpline2D::CubicSpline2D(const VectorXd &x_data, const VectorXd &y_data, double max_dist_proj_params) {
    s_vector = cumulativeSum(calcLineDistances(x_data, y_data));
    sx = CubicSpline(s_vector, x_data);
    sy = CubicSpline(s_vector, y_data);
    max_dist_proj = max_dist_proj_params;
}

VectorXd CubicSpline2D::calcLineDistances(const VectorXd &x_data, const VectorXd &y_data) {
    double dx, dy; int i;
    VectorXd arr = VectorXd::Zero(x_data.size());
    for (i = 1; i < x_data.size(); i++)  {
        dx = x_data(i) - x_data(i-1);
        dy = y_data(i) - y_data(i-1);
        arr(i) = hypot(dx, dy);
    }
    return arr;
}

VectorXd CubicSpline2D::cumulativeSum(const VectorXd &xy_data) {
    double sum = 0.0; int i;
    VectorXd arr = VectorXd::Zero(xy_data.size());
    for (i = 0; i <= xy_data.size(); i++) {
        arr(i) = sum + arr(i);
    }
    return arr;
}

double CubicSpline2D::constrainInput(double s) const {
    if (s < s_vector(0)) {
        return s_vector(0);
    }
    if (s > s_vector(s_vector.size()-1)) {
        return s_vector(s_vector.size()-1);
    }
    return s;
}

Vector2d CubicSpline2D::getPosition(double s) const {
    s = constrainInput(s);
    Vector2d position = Vector2d::Zero();
    position(0) = sx.getPosition(s);
    position(1) = sy.getPosition(s);
    return position;
}

Vector2d CubicSpline2D::getDerivative(double s) const {
    s = constrainInput(s);
    Vector2d velocity = Vector2d::Zero();
    velocity(0) = sx.getDerivative(s);
    velocity(1) = sy.getDerivative(s);
    return velocity;
}

Vector2d CubicSpline2D::getSecondDerivative(double s) const {
    s = constrainInput(s);
    Vector2d acceleration = Vector2d::Zero();
    acceleration(0) = sx.getSecondDerivative(s);
    acceleration(1) = sy.getSecondDerivative(s);
    return acceleration;
}

double CubicSpline2D::getLength() const { return s_vector(s_vector.size()-1); }

double CubicSpline2D::projectOnSpline(const State &xk) const {
    Vector2d pos;
    pos(IndexMap.X) = xk(IndexMap.X);
    pos(IndexMap.Y) = xk(IndexMap.Y);
    double s_guess = xk(IndexMap.s);
    Vector2d pos_path = getPosition(s_guess);

    double s_opt = s_guess;
    double dist = (pos - pos_path).norm();

    if (dist >= max_dist_proj) {
        ArrayXd diff_x_all = sx.ft_spline.array() - pos(0);
        ArrayXd diff_y_all = sy.ft_spline.array() - pos(1);
        ArrayXd dist_square = diff_x_all.square() + diff_y_all.square();
        vector<double> dist_square_vec(dist_square.data(), dist_square.data() + dist_square.size());
        auto min_iter = std::min_element(dist_square_vec.begin(), dist_square_vec.end());
        s_opt = s_vector(std::distance(dist_square_vec.begin(), min_iter));
    }

    double s_old = s_opt;
    Vector2d ds_path, dds_path, diff;
    double jac, hessian;
    for (int i = 0; i < 20; i++) {
        pos_path = getPosition(s_opt);
        ds_path = getDerivative(s_opt);
        dds_path = getSecondDerivative(s_opt);
        diff = pos_path - pos;
        jac = (2.0 * diff(0) * ds_path(0)) + (2.0 * diff(1) * ds_path(1));
        hessian = 2.0 * ds_path(0) * ds_path(0) + 2.0 * diff(0) * dds_path(0) +
                  2.0 * ds_path(1) * ds_path(1) + 2.0 * diff(1) * dds_path(1);
        // Newton method
        s_opt -= jac / hessian;
        s_opt = constrainInput(s_opt);

        if (abs(s_old - s_opt) <= 1e-5) {
            return s_opt;
        }
        s_old = s_opt;
    }
    // something is strange if it did not converge within 20 iterations, give back the initial guess
    return s_guess;
}
