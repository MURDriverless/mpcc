//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_CONSTRAINT_TYPES_H
#define MPCC_CONSTRAINT_TYPES_H

#include <Eigen/Dense>
#include "../models/state.h"

using Eigen::Vector2d;
using Eigen::Matrix;


/* Polytopic constraint constants */
//
// Number of polytopic constraints:
// 1. Track constraint (lateral deviation)
// 2. Rear tire forces ellipse constraint
// 3. Front tire forces ellipse constraint
// 4. Rear alpha constraint
// 5. Front alpha constraint
// while state and input constraints are termed as lower and upper "bounds"
#define NPC 5
// Number of soft constraints (we have 5 for all our polytopic constraints):
#define NS 5
// Infinity
static constexpr double INF = 1E5;


/* Polytopic constraint types */
typedef Eigen::Matrix<double, NPC, NX> C_MPC;  // Jacobian of all constraints versus state
typedef Eigen::Matrix<double, 1, NX> C_i_MPC;  // Jacobian of one constraint versus state
typedef Eigen::Matrix<double, NPC, NU> D_MPC;  // Jacobian of all constraints versus input
typedef Eigen::Matrix<double, NPC, 1> d_MPC;  // limit of all constraints (min or max)

struct Constraint1D {
    C_i_MPC C_i;
    const double lower_bound;
    const double upper_bound;
};

struct ConstraintsMatrix {
    // dl <= C*xk + D*uk <= du
    C_MPC C;
    D_MPC D;
    d_MPC dl;
    d_MPC du;
};


/* Polytopic constraint indices */
struct ConstraintIndexStruct {
    int track = 0;  // constrain the car within the track
    int tire_rear = 1;   // rear tire force ellipsis
    int tire_front = 2;  // front tire force ellipsis
    int alpha_rear = 3;  // rear tire slip angle
    int alpha_front = 4; // front tire slip angle
};

static const ConstraintIndexStruct ConstraintIndex;

#endif //MPCC_CONSTRAINT_TYPES_H
