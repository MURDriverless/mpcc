//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_MPC_TYPES_H
#define MPCC_MPC_TYPES_H

#include <array>
#include "../bounds/bounds_types.h"
#include "../constraints/constraint_types.h"
#include "../costs/cost_types.h"
#include "../models/state.h"

// Horizon length constant
static constexpr int N = 30;

struct OptVariable {
    State xk;
    Input uk;
};

struct OptStage {
    LinModelMatrix model;
    CostMatrix costs;
    ConstraintsMatrix constraints;

    Bounds_x x_lower;
    Bounds_x x_upper;

    Bounds_u u_lower;
    Bounds_u u_upper;

    Bounds_s s_lower;
    Bounds_s s_upper;

    // nx    -> number of states
    // nu    -> number of inputs
    // nbx   -> number of bounds on x
    // nbu   -> number of bounds on u
    // ng    -> number of polytopic constratins
    // ns    -> number of soft constraints
    int nx, nu, nbx, nbu, ng, ns;
};

struct OptSolution {
    const Input u0;
    const std::array<OptVariable, N+1> opt_vars;
    const double exec_time;
};

#endif //MPCC_MPC_TYPES_H
