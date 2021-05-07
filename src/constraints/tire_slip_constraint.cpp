//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "tire_slip_constraint.h"

Constraint1D TireSlipConstraint::getRearAlphaConstraint(const DynamicBicycleModel &model, const State &xk) {
    // 0. Unpack parameters
    const double vx = xk(IndexMap.vx);
    const double vy = xk(IndexMap.vy);
    const double wz = xk(IndexMap.wz);
    const double lr = model.params.lr;

    // 1. Calculate Jacobian terms (use Matlab)
    // alpha_r = -atan((vy-wz*lr)/vx)
    const double dalpha_dvx = (vy-lr*wz) / (pow(vy-lr*wz, 2) + pow(vx, 2));
    const double dalpha_dvy = -1 / (pow(vy-lr*wz, 2)/vx + vx);
    const double dalpha_dwz = lr / (pow(vy-lr*wz, 2)/vx + vx);

    // 2. Construct Jacobian
    C_i_MPC J_alpha = C_i_MPC::Zero();
    J_alpha(IndexMap.vx) = dalpha_dvx;
    J_alpha(IndexMap.vy) = dalpha_dvy;
    J_alpha(IndexMap.wz) = dalpha_dwz;

    // 3. Compute bounds
    // -alpha_max <= J_alpha*(x - x0) + alpha <= alpha_max
    // -alpha_max <= J_alpha*x + (-J_alpha*x0 + alpha) <= alpha_max
    // -alpha_max + J_alpha*x0 - alpha <= J_alpha*x <= alpha_max + J_alpha*x0 - alpha
    const double alpha = -atan((vy-wz*lr)/vx);
    const double alpha_max = model.params.max_alpha;
    const double lower_bound = -alpha_max + J_alpha*xk - alpha;
    const double upper_bound = alpha_max + J_alpha*xk - alpha;

    return { J_alpha, lower_bound, upper_bound };
}

Constraint1D TireSlipConstraint::getFrontAlphaConstraint(const DynamicBicycleModel &model, const State &xk) {
    // 0. Unpack parameters
    const double vx = xk(IndexMap.vx);
    const double vy = xk(IndexMap.vy);
    const double wz = xk(IndexMap.wz);
    const double steering_angle  = xk(IndexMap.steering_angle);
    const double lf = model.params.lf;

    // 1. Calculate Jacobian terms (use Matlab)
    // alpha_f = -atan((vy+wz*lf)/vx) + steering_angle.
    const double dalpha_dvx = (vy+lf*wz) / (pow(vy+lf*wz, 2) + pow(vx, 2));
    const double dalpha_dvy = -vx / (pow(vy+lf*wz, 2) + pow(vx, 2));
    const double dalpha_dwz = -(vx*lf) / (pow(vy+lf*wz, 2) + pow(vx, 2));
    const double dalpha_dsteering_angle = 1.0;

    // 2. Construct Jacobian
    C_i_MPC J_alpha = C_i_MPC::Zero();
    J_alpha(IndexMap.vx) = dalpha_dvx;
    J_alpha(IndexMap.vy) = dalpha_dvy;
    J_alpha(IndexMap.wz) = dalpha_dwz;
    J_alpha(IndexMap.steering_angle) = dalpha_dsteering_angle;

    // 3. Compute bounds
    // -alpha_max <= J_alpha*(x - x0) + alpha <= alpha_max
    // -alpha_max <= J_alpha*x + (-J_alpha*x0 + alpha) <= alpha_max
    // -alpha_max + J_alpha*x0 - alpha <= J_alpha*x <= alpha_max + J_alpha*x0 - alpha
    const double alpha = -atan((vy+wz*lf)/vx) + steering_angle;
    const double alpha_max = model.params.max_alpha;
    const double lower_bound = -alpha_max + J_alpha*xk - alpha;
    const double upper_bound = alpha_max + J_alpha*xk - alpha;

    return { J_alpha, lower_bound, upper_bound };
}
