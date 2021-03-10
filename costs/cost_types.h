//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_COST_TYPES_H
#define MPCC_COST_TYPES_H

#include "../models/state.h"

// Number of soft constraints (we have 5 for all our constraints):
#define NS 5

// Quadratic cost
typedef Matrix<double, NX, NX> Q_MPC;  // Error cost
typedef Matrix<double, NU, NU> R_MPC;  // Input cost
typedef Matrix<double, NX, NX> S_MPC;  // Soft constraints cost

// Linear cost
typedef Matrix<double, NX, 1> q_MPC;
typedef Matrix<double, NU, 1> r_MPC;

// Soft constraints
typedef Eigen::Matrix<double, NS, NS> Z_MPC;
typedef Eigen::Matrix<double, NS, 1> z_MPC;

struct CostMatrix {
    Q_MPC Q;
    R_MPC R;
    S_MPC S;
    q_MPC q;
    r_MPC r;
    Z_MPC Z;
    z_MPC z;
};

// Return type for contouring, input, soft constraints and slip angle cost functions
template <class QuadCost, class LinCost>
struct CostTerm {
    QuadCost quad;
    LinCost lin;
};

#endif //MPCC_COST_TYPES_H
