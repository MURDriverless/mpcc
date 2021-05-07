//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "constraint_manager.h"

ConstraintManager::ConstraintManager(const DynamicBicycleModel &model_args) {
    model = model_args;
}

ConstraintsMatrix ConstraintManager::getConstraints(const Track &track, const State &xk) const {
    const Constraint1D track_constraint = TrackConstraint::getTrackConstraint(track, xk);
    const Constraint1D tire_constraint_rear = TireEllipsisConstraint::getRearTireConstraint(model, xk);
    const Constraint1D tire_constraint_front = TireEllipsisConstraint::getFrontTireConstraint(model, xk);
    const Constraint1D alpha_constraint_rear = TireSlipConstraint::getRearAlphaConstraint(model, xk);
    const Constraint1D alpha_constraint_front = TireSlipConstraint::getFrontAlphaConstraint(model, xk);

    C_MPC C;
    d_MPC dl;
    d_MPC du;

    // 1. Track constraint
    C.row(ConstraintIndex.track) = track_constraint.C_i;
    dl(ConstraintIndex.track) = track_constraint.lower_bound;
    du(ConstraintIndex.track) = track_constraint.upper_bound;

    // 2. Rear tire force ellipsis
    C.row(ConstraintIndex.tire_rear) = tire_constraint_rear.C_i;
    dl(ConstraintIndex.tire_rear) = tire_constraint_rear.lower_bound;
    du(ConstraintIndex.tire_rear) = tire_constraint_rear.upper_bound;

    // 3. Front tire force ellipsis
    C.row(ConstraintIndex.tire_front) = tire_constraint_front.C_i;
    dl(ConstraintIndex.tire_front) = tire_constraint_front.lower_bound;
    du(ConstraintIndex.tire_front) = tire_constraint_front.upper_bound;

    // 4. Rear tire slip angle
    C.row(ConstraintIndex.alpha_rear) = alpha_constraint_rear.C_i;
    dl(ConstraintIndex.alpha_rear) = alpha_constraint_rear.lower_bound;
    du(ConstraintIndex.alpha_rear) = alpha_constraint_rear.upper_bound;

    // 5. Front tire slip angle
    C.row(ConstraintIndex.alpha_front) = alpha_constraint_front.C_i;
    dl(ConstraintIndex.alpha_front) = alpha_constraint_front.lower_bound;
    du(ConstraintIndex.alpha_front) = alpha_constraint_front.upper_bound;

    return { C, D_MPC::Zero(), dl, du };
}
