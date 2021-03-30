//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_BOUNDS_TYPES_H
#define MPCC_BOUNDS_TYPES_H

#include <Eigen/Dense>
#include "../constraints/constraint_types.h"
#include "../models/state.h"

// Bounds matrices
typedef Eigen::Matrix<double, NX, 1> Bounds_x;
typedef Eigen::Matrix<double, NU, 1> Bounds_u;
typedef Eigen::Matrix<double, NS, 1> Bounds_s;

#endif //MPCC_BOUNDS_TYPES_H
