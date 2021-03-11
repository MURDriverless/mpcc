//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "cost_manager.h"

CostManager::CostManager(const CostParams &cost_params, const ModelParams &model_params) {
    costParams = cost_params;
    modelParams = model_params;
}

CostMatrix CostManager::getCost(const CubicSpline2D &path, const State &xk) const {
    CostTerm<Q_MPC, q_MPC> contouring = getContouringCost(path, xk);
    CostTerm<Q_MPC, q_MPC> raw_input = getRawInputCost();
    CostTerm<R_MPC, r_MPC> input_change = getInputChangeCost();
    CostTerm<Z_MPC, z_MPC> soft_constraints = getSoftConstraintsCost();
    CostTerm<Q_MPC, q_MPC> slip_angle = getBetaCost(xk);

    // Q
    Q_MPC Q_not_sym = contouring.quad_cost + raw_input.quad_cost + slip_angle.quad_cost;
    Q_not_sym = 2.0 * Q_not_sym;
    Q_MPC Q = 0.5 * (Q_not_sym.transpose() + Q_not_sym);

    // R
    R_MPC R = input_change.quad_cost;
    // Solver expects 0.5*uT*R*u
    R = 2.0 * R;

    // S (polytopic cost) will be 0 as it is not used
    S_MPC S = S_MPC::Zero();

    // q
    q_MPC q = contouring.lin_cost + raw_input.lin_cost + slip_angle.lin_cost;
    // Not sure about the line below, this was implemented in AL-MPCC
    q = q + (xk.adjoint()*Q).adjoint();

    // r, which should be zero
    r_MPC r = input_change.lin_cost;

    // Z
    Z_MPC Z = soft_constraints.quad_cost;
    // Similar to Q and R, quad cost has to be multiplied by 2.0
    Z = 2.0 * Z;

    // z
    z_MPC z = soft_constraints.lin_cost;

    return { Q, R, S, q, r, Z, z };
}

CostTerm<Q_MPC, q_MPC> CostManager::getContouringCost(const CubicSpline2D &path, const State &xk) const {
    const ContouringError contour_error = ContouringErrorDelegate::getContouringError(path, xk);
    const Matrix<double, 2, 1> error_bar = contour_error.error - contour_error.d_error * xk;

    Matrix<double, 2, 2> contouring_weights = Matrix<double, 2, 2>::Zero();
    contouring_weights(0, 0) = costParams.q_c;
    contouring_weights(1, 1) = costParams.q_l;

    // TODO: Link Github issue or post in README on the error linearisation maths
    Q_MPC Q = contour_error.d_error.transpose() * contouring_weights * contour_error.d_error;
    q_MPC q = 2 * error_bar.transpose() * contouring_weights * contour_error.d_error;

    // Maximise progression via virtual input
    Q(IndexMap.vs, IndexMap.vs) = -costParams.q_vs;

    return { Q, q };
}

CostTerm<Q_MPC, q_MPC> CostManager::getRawInputCost() const {
    // Raw input cost is linear, so there are only quadratic terms
    Q_MPC Q = Q_MPC::Zero();
    Q(IndexMap.accel_D, IndexMap.accel_D) = costParams.r_accel_D;
    Q(IndexMap.steering_angle, IndexMap.steering_angle) = costParams.r_steering_angle;
    Q(IndexMap.vs, IndexMap.vs) = costParams.r_vs;
    return { Q, q_MPC::Zero() };
}

CostTerm<R_MPC, r_MPC> CostManager::getInputChangeCost() const {
    // Input change cost is also linear, so there are only quadratic terms
    R_MPC R = R_MPC::Zero();
    R(IndexMap.d_accel_D, IndexMap.d_accel_D) = costParams.r_d_accel_D;
    R(IndexMap.d_steering_angle, IndexMap.d_steering_angle) = costParams.r_d_steering_angle;
    R(IndexMap.d_vs, IndexMap.d_vs) = costParams.r_d_vs;
    return { R, r_MPC::Zero() };
}

CostTerm<Z_MPC, z_MPC> CostManager::getSoftConstraintsCost() const {
    Z_MPC Z_cost = Z_MPC::Zero();
    z_MPC z_cost = z_MPC::Zero();
    // Quadratic cost
    Z_cost(ConstraintIndex.track, ConstraintIndex.track) = costParams.sc_quad_track;
    Z_cost(ConstraintIndex.tire_rear, ConstraintIndex.tire_rear) = costParams.sc_quad_tire;
    Z_cost(ConstraintIndex.tire_front, ConstraintIndex.tire_front) = costParams.sc_quad_tire;
    Z_cost(ConstraintIndex.alpha_rear, ConstraintIndex.alpha_rear) = costParams.sc_quad_alpha;
    Z_cost(ConstraintIndex.alpha_front, ConstraintIndex.alpha_front) = costParams.sc_quad_alpha;
    // Linear cost
    z_cost(ConstraintIndex.track) = costParams.sc_lin_track;
    z_cost(ConstraintIndex.tire_rear) = costParams.sc_lin_tire;
    z_cost(ConstraintIndex.tire_front) = costParams.sc_lin_tire;
    z_cost(ConstraintIndex.alpha_rear) = costParams.sc_lin_alpha;
    z_cost(ConstraintIndex.alpha_front) = costParams.sc_lin_alpha;
    return { Z_cost, z_cost };
}

CostTerm<Q_MPC, q_MPC> CostManager::getBetaCost(const State &xk) const {
    LinearisedVar beta_kin = SlipAngleDelegate::getBetaKin(xk, modelParams.lf, modelParams.lr);
    LinearisedVar beta_dyn = SlipAngleDelegate::getBetaDyn(xk);

    const double beta_bar = beta_kin.bar - beta_dyn.bar;
    const Matrix<double, 1, NX> d_beta_kin = beta_kin.jac;
    const Matrix<double, 1, NX> d_beta_dyn = beta_dyn.jac;
    const Matrix<double, 1, NX> d_beta = d_beta_kin - d_beta_dyn;

    Q_MPC Q = d_beta.transpose() * costParams.q_beta * d_beta;
    q_MPC q = 2 * beta_bar * costParams.q_beta * d_beta;

    return { Q, q };
}
