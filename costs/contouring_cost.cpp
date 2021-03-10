//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "contouring_cost.h"

ContouringCost::ContouringCost() {
    costParams = nullptr;
    modelParams = nullptr;
};

ContouringCost::ContouringCost(CostParams *cost_params, ModelParams *model_params) {
    costParams = cost_params;
    modelParams = model_params;
}

CostTerm<Q_MPC, q_MPC> ContouringCost::getCost(const CubicSpline2D &path, const State &xk) const {
    const ContouringError contour_error = getContouringError(path, xk);
    const Matrix<double, 2, 1> error_bar = contour_error.error - contour_error.d_error * xk;

    Matrix<double, 2, 2> contouring_weights = Matrix<double, 2, 2>::Zero();
    contouring_weights(0, 0) = costParams->q_c;
    contouring_weights(1, 1) = costParams->q_l;

    // TODO: Link Github issue or post in README on the error linearisation maths
    Q_MPC Q = contour_error.d_error.transpose() * contouring_weights * contour_error.d_error;
    q_MPC q = 2 * error_bar.transpose() * contouring_weights * contour_error.d_error;

    // Maximise progression via virtual input
    Q(IndexMap.vs, IndexMap.vs) = -costParams->q_vs;

    return { Q, q };
}

RefPoint ContouringCost::getRefPoint(const CubicSpline2D &path, const State &xk) {
    const double theta_path = xk(IndexMap.s);
    const Vector2d x_y = path.getPosition(theta_path);
    const Vector2d dx_dy = path.getDerivative(theta_path);
    const Vector2d ddx_ddy = path.getSecondDerivative(theta_path);

    // Unpack variables
    const double x = x_y(0);
    const double y = x_y(1);
    const double dx = dx_dy(0);
    const double dy = dx_dy(1);
    const double yaw = atan2(dx_dy(1), dx_dy(0));

    // Note that dyaw is not angular rate of change of yaw, because dyaw is partial derivative (we need time
    // for angular rate of change). Instead, we now have the curvature of the path.
    // Formula for curvature = |dx*ddy + dy*ddx| / (dx^2 + dy^2)^(3/2)
    const double ddx = ddx_ddy(0);
    const double ddy = ddx_ddy(1);
    const double dyaw_numerator = dx*ddy + dy*ddx;
    const double dyaw_denominator = pow(dx*dx + dy*dy, 1.5);
    const double dyaw = dyaw_numerator / dyaw_denominator;

    return { x, y, dx, dy, yaw, dyaw };
}

ContouringError ContouringCost::getContouringError(const CubicSpline2D &path, const State &xk) {
    const RefPoint ref = getRefPoint(path, xk);
    Matrix<double, 2, 1> error = Matrix<double, 2, 1>::Zero();
    Matrix<double, 2, NX> d_error = Matrix<double, 2, NX>::Zero();

    // Exact error
    error(0) = sin(ref.yaw)*(xk(IndexMap.X) - ref.x) - cos(ref.yaw)*(xk(IndexMap.Y) - ref.y);
    error(1) = -cos(ref.yaw)*(xk(IndexMap.X) - ref.x) - sin(ref.yaw)*(xk(IndexMap.Y) - ref.y);

    // d_contour_error and d_lag_error are partial derivatives of the errors with respect to the path parameter
    const double d_contour_error = ref.dyaw*cos(ref.yaw)*(xk(IndexMap.X) - ref.x) - ref.dx*sin(ref.yaw) +
                                   ref.dyaw*sin(ref.yaw)*(xk(IndexMap.Y) - ref.y) + ref.dy*cos(ref.yaw);

    const double d_lag_error = ref.dyaw*sin(ref.yaw)*(xk(IndexMap.X) - ref.x) + ref.dx*cos(ref.yaw) -
                               ref.dyaw*cos(ref.yaw)*(xk(IndexMap.Y) - ref.y) + ref.dy*sin(ref.yaw);

    // Jacobian of contouring error
    d_error(0, IndexMap.X) = sin(ref.yaw);
    d_error(0, IndexMap.Y) = -cos(ref.yaw);
    d_error(0, IndexMap.s) = d_contour_error;

    // Jacobian of lag error
    d_error(1, IndexMap.X) = -cos(ref.yaw);
    d_error(1, IndexMap.Y) = -sin(ref.yaw);
    d_error(1, IndexMap.s) = d_lag_error;

    return { error, d_error };
}
