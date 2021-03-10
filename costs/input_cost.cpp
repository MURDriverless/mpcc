//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "input_cost.h"

InputCost::InputCost() {
    costParams = nullptr;
}

InputCost::InputCost(CostParams *cost_params) {
    costParams = cost_params;
}

CostTerm<Q_MPC, q_MPC> InputCost::getRawInputCost() const {
    // Raw input cost is linear, so there are only quadratic terms
    Q_MPC Q = Q_MPC::Zero();
    Q(IndexMap.accel_D, IndexMap.accel_D) = costParams->r_accel_D;
    Q(IndexMap.steering_angle, IndexMap.steering_angle) = costParams->r_steering_angle;
    Q(IndexMap.vs, IndexMap.vs) = costParams->r_vs;
    return { Q, q_MPC::Zero() };
}

CostTerm<R_MPC, r_MPC> InputCost::getInputChangeCost() const {
    // Input change cost is also linear, so there are only quadratic terms
    R_MPC R = R_MPC::Zero();
    R(IndexMap.d_accel_D, IndexMap.d_accel_D) = costParams->r_d_accel_D;
    R(IndexMap.d_steering_angle, IndexMap.d_steering_angle) = costParams->r_d_steering_angle;
    R(IndexMap.d_vs, IndexMap.d_vs) = costParams->r_d_vs;
    return { R, r_MPC::Zero() };
}
