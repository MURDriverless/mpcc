//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "tire_ellipsis_constraint.h"

Constraint1D TireEllipsisConstraint::getRearTireConstraint(const DynamicBicycleModel &model, const State &xk) {
    // 0a. Unpack model params
    const double e_long = model.params.e_long;
    const double e_eps = model.params.e_eps;
    const double Dr = model.params.Dr;
    // 0b. Set up helper variables
    TireForces F_rear = model.getForceRear(xk);
    TireForcesDerivatives dF_rear = model.getForceRearDerivatives(xk);
    double Fn_rear = model.params.lf / (model.params.lf + model.params.lr) * model.params.m * model.params.g;
    double e_long_Fn_2 = pow(e_long / Fn_rear, 2);
    double Fn_inv_2 = pow(1.0 / Fn_rear, 2);

    // 1. Calculate Jacobian terms
    // Check dynamic_bicycle.cpp to identify non-zero derivatives
    // TC (Tire Constraints) = (e_long*(F_{rear,x}/Fn_rear))^2 + (F_{rear,y}/Fn_rear)^2
    const double dTC_dvx = e_long_Fn_2 * 2.0 * F_rear.Fx * dF_rear.dFx_vx +
                           Fn_inv_2 * 2.0 * F_rear.Fy * dF_rear.dFy_vx;
    const double dTC_dvy = Fn_inv_2 * 2.0 * F_rear.Fy * dF_rear.dFy_vy;
    const double dTC_dwz = Fn_inv_2 * 2.0 * F_rear.Fy * dF_rear.dFy_wz;
    const double dTC_daccel_D = e_long_Fn_2 * 2.0 * F_rear.Fx * dF_rear.dFx_accel_D;

    // 2. Construct Jacobian
    C_i_MPC J_TC = C_i_MPC::Zero();
    J_TC(IndexMap.vx) = dTC_dvx;
    J_TC(IndexMap.vy) = dTC_dvy;
    J_TC(IndexMap.wz) = dTC_dwz;
    J_TC(IndexMap.accel_D) = dTC_daccel_D;

    // 3. Compute constraint bounds
    // 0 <= J_TC*(x - x0) + TC <= F_max
    // 0 <= J_TC*x + (-J_TC*x0 + TC) <= F_max
    // J_TC*x0 - TC <= J_TC*x <= F_max + J_TC*x0 - TC
    // where x is the decision variable and x0 is our linearisation point (xk in our definition).
    const double TC = pow(e_long*F_rear.Fx/Fn_rear, 2) + pow(F_rear.Fy/Fn_rear, 2);
    const double F_max = pow(e_eps * Dr, 2);
    const double lower_bound = J_TC*xk - TC;
    const double upper_bound = J_TC*xk + F_max - TC;

    return { J_TC, lower_bound, upper_bound };
}

Constraint1D TireEllipsisConstraint::getFrontTireConstraint(const DynamicBicycleModel &model, const State &xk) {
    // 0a. Unpack model params
    const double e_long = model.params.e_long;
    const double e_eps = model.params.e_eps;
    const double Df = model.params.Df;
    // 0b. Set up helper variables
    TireForces F_front = model.getForceFront(xk);
    TireForcesDerivatives dF_front = model.getForceFrontDerivatives(xk);
    double Fn_front = model.params.lr / (model.params.lf + model.params.lr) * model.params.m * model.params.g;
    double Fn_inv_2 = pow(1.0 / Fn_front, 2);

    // 1. Calculate Jacobian terms
    // Check dynamic_bicycle.cpp to identify non-zero derivatives
    // TC (Tire Constraints) = (param_.e_long*(F_{front,x}/Fn_rear))^2 + (F_{front,y}/Fn_rear)^2
    const double dTC_dvx = Fn_inv_2 * 2.0 * F_front.Fy * dF_front.dFy_vx;
    const double dTC_dvy = Fn_inv_2 * 2.0 * F_front.Fy * dF_front.dFy_vy;
    const double dTC_dwz = Fn_inv_2 * 2.0 * F_front.Fy * dF_front.dFy_wz;
    const double dTC_dsteering_angle = 2.0 * F_front.Fy * dF_front.dFy_steering_angle;

    // 2. Construct Jacobian
    C_i_MPC J_TC = C_i_MPC::Zero();
    J_TC(IndexMap.vx) = dTC_dvx;
    J_TC(IndexMap.vy) = dTC_dvy;
    J_TC(IndexMap.wz) = dTC_dwz;
    J_TC(IndexMap.steering_angle) = dTC_dsteering_angle;

    // 3. Compute constraint bounds
    // 0 <= J_TC*(x - x0) + TC <= F_max
    // 0 <= J_TC*x + (-J_TC*x0 + TC) <= F_max
    // J_TC*x0 - TC <= J_TC*x <= F_max + J_TC*x0 - TC
    // where x is the decision variable and x0 is our linearisation point (xk in our definition).
    const double TC = pow(e_long * F_front.Fx / Fn_front, 2) + pow(F_front.Fy / Fn_front, 2);
    const double F_max = pow(e_eps * Df, 2);
    const double lower_bound = J_TC*xk - TC;
    const double upper_bound = J_TC*xk + F_max - TC;

    return { J_TC, lower_bound, upper_bound };
}
