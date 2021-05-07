//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_STATE_H
#define MPCC_STATE_H

#include <cmath>
#include <Eigen/Dense>

using Eigen::Matrix;

#define NX 10
#define NU 3

struct StateInputIndex {
    /* State indices */
    int X = 0;              // global x position
    int Y = 1;              // global y position
    int yaw = 2;            // global heading of car
    int vx = 3;             // linear velocity in local frame
    int vy = 4;             // linear velocity in local frame
    int wz = 5;             // angular velocity around z-axis (yaw)
    int s = 6;              // path parameter, track progress (virtual state)
    int accel_D = 7;        // duty cycle of acceleration: [-1, 1]
    int steering_angle = 8; // steering angle in radians
    int vs = 9;             // "velocity" of path parameter, i.e. rate of change of track progress

    /* Input indices */
    int d_accel_D = 0;
    int d_steering_angle = 1;
    int d_vs = 2;
};

// State vector
typedef Matrix<double, NX, 1> State;

// Input vector
typedef Matrix<double, NU, 1> Input;

// Linearised state matrix
typedef Matrix<double, NX, NX> A_MPC;

// Linearised input matrix
typedef Matrix<double, NX, NU> B_MPC;

// Linearised offset such that the dynamics forms -> xk1 = A*xk + B*uk + g
typedef Matrix<double, NX, 1> g_MPC;

// Contrainer to store linear model matrices
struct LinModelMatrix {
    A_MPC A;
    B_MPC B;
    g_MPC g;
};

static const StateInputIndex IndexMap;

namespace mpcc {
    State vxNonZero(const State &xk, double min_vx);
    State constrainState(const State &xk, double path_length);
}

#endif //MPCC_STATE_H
