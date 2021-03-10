//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_BINARY_SEARCH_H
#define MPCC_BINARY_SEARCH_H

#include <iostream>
#include <Eigen/Dense>

using Eigen::VectorXd;

namespace mpcc {
    int binary_search_left(const VectorXd &arr, double x);
}

#endif //MPCC_BINARY_SEARCH_H
