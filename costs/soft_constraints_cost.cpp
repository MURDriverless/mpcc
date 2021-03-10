//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "soft_constraints_cost.h"

SoftConstraintsCost::SoftConstraintsCost() {
    costParams = nullptr;
}

SoftConstraintsCost::SoftConstraintsCost(CostParams *cost_params) {
    costParams = cost_params;
}

CostTerm<Z_MPC, z_MPC> SoftConstraintsCost::getCost() const {
    Z_MPC Z_cost = Z_MPC::Zero();
    z_MPC z_cost = z_MPC::Zero();
    // Quadratic cost
    Z_cost(ConstraintMap.track, ConstraintMap.track) = costParams->sc_quad_track;
    Z_cost(ConstraintMap.tire_rear, ConstraintMap.tire_rear) = costParams->sc_quad_tire;
    Z_cost(ConstraintMap.tire_front, ConstraintMap.tire_front) = costParams->sc_quad_tire;
    Z_cost(ConstraintMap.alpha_rear, ConstraintMap.alpha_rear) = costParams->sc_quad_alpha;
    Z_cost(ConstraintMap.alpha_front, ConstraintMap.alpha_front) = costParams->sc_quad_alpha;
    // Linear cost
    z_cost(ConstraintMap.track) = costParams->sc_lin_track;
    z_cost(ConstraintMap.tire_rear) = costParams->sc_lin_tire;
    z_cost(ConstraintMap.tire_front) = costParams->sc_lin_tire;
    z_cost(ConstraintMap.alpha_rear) = costParams->sc_lin_alpha;
    z_cost(ConstraintMap.alpha_front) = costParams->sc_lin_alpha;
    return { Z_cost, z_cost };
}
