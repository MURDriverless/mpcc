//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_BOUNDS_MANAGER_H
#define MPCC_BOUNDS_MANAGER_H

#include <string>
#include <fstream>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include "../constraints/constraint_types.h"
#include "../models/state.h"

// Bounds matrices
typedef Eigen::Matrix<double, NX, 1> Bounds_x;
typedef Eigen::Matrix<double, NU, 1> Bounds_u;
typedef Eigen::Matrix<double, NS, 1> Bounds_s;

class BoundsManager {
public:
    BoundsManager() = default;
    explicit BoundsManager(const std::string &file_path);

    Bounds_x x_lower;
    Bounds_x x_upper;
    Bounds_u u_lower;
    Bounds_u u_upper;
    Bounds_s s_lower;
    Bounds_s s_upper;
};

#endif //MPCC_BOUNDS_MANAGER_H
