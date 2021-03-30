//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_MPC_PARAMS_H
#define MPCC_MPC_PARAMS_H

#include <string>
#include <fstream>
#include <nlohmann/json.hpp>

class MPCParams {
public:
    MPCParams() = default;
    explicit MPCParams(const std::string &file_path);

    int n_sqp;
    double sqp_mixing;
    int n_non_solves;
    int n_no_solves_sqp;
    int n_reset;
    double Ts;
};

#endif //MPCC_MPC_PARAMS_H
