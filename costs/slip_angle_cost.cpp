//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "slip_angle_cost.h"

SlipAngleCost::SlipAngleCost() {
    costParams = nullptr;
    modelParams = nullptr;
}

SlipAngleCost::SlipAngleCost(CostParams *cost_params, ModelParams *model_params) {
    costParams = cost_params;
    modelParams = model_params;
}

CostTerm<Q_MPC, q_MPC> SlipAngleCost::getCost(const State &xk) const {
    LinearisedVar beta_kin = getBetaKin(xk, modelParams->lf, modelParams->lr);
    LinearisedVar beta_dyn = getBetaDyn(xk);

    const double beta_bar = beta_kin.bar - beta_dyn.bar;
    const Matrix<double, 1, NX> d_beta_kin = beta_kin.jac;
    const Matrix<double, 1, NX> d_beta_dyn = beta_dyn.jac;
    const Matrix<double, 1, NX> d_beta = d_beta_kin - d_beta_dyn;

    Q_MPC Q = d_beta.transpose() * costParams->q_beta * d_beta;
    q_MPC q = 2 * beta_bar * costParams->q_beta * d_beta;

    return { Q, q };
}

LinearisedVar SlipAngleCost::getBetaKin(const State &xk, double lf, double lr) {
    const double delta = xk(IndexMap.steering_angle);

    // Exact value
    const double beta_kin = atan((lr*tan(delta)) / (lf*lf + lr*lr));

    // Partial derivatives with respect to steering angle (delta), computed symbolic-ally using Matlab
    Matrix<double, 1, NX> d_beta_kin = Matrix<double, 1, NX>::Zero();
    // Define shorthand for a simpler formula
    const double l_ratio = lr / (lf + lr);
    // delta: l_ratio * (tan(delta)^2 + 1) / (l_ratio^2 * tan(delta)^2 + 1)
    d_beta_kin(0, IndexMap.steering_angle) = l_ratio*(pow(tan(delta), 2)+1.0) /
                                             (pow(l_ratio, 2)*pow(tan(delta), 2) + 1.0);

    const double beta_kin_bar = beta_kin - d_beta_kin*xk;

    return { beta_kin_bar, d_beta_kin };
}

LinearisedVar SlipAngleCost::getBetaDyn(const State &xk) {
    const double vx = xk(IndexMap.vx);
    const double vy = xk(IndexMap.vy);

    // Exact value
    const double beta_dyn = atan(vy / vx);

    // Partial derivatives with respect to vx and vy, computed symbolically using Matlab
    Matrix<double, 1, NX> d_beta_dyn = Matrix<double, 1, NX>::Zero();
    // vx: -vy/(vx^2 + vy^2)
    d_beta_dyn(0, IndexMap.vx) = (-vy) / (vx*vx + vy*vy);
    // vy: vx/(vx^2 + vy^2)
    d_beta_dyn(0, IndexMap.vy) = (vx) / (vx*vx + vy*vy);

    // Setpoint value from linearisation
    const double beta_dyn_bar = beta_dyn - d_beta_dyn*xk;

    return { beta_dyn_bar, d_beta_dyn };
}
