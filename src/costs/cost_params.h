//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_COST_PARAMS_H
#define MPCC_COST_PARAMS_H

#include <string>
#include <fstream>
#include <nlohmann/json.hpp>

class CostParams {
public:
    CostParams() = default;
    explicit CostParams(const std::string &file_path);

    double q_c;
    double q_l;
    double q_vs;
    double q_beta;  // vehicular slip angle regularisation cost

    double r_accel_D;
    double r_steering_angle;
    double r_vs;

    double r_d_accel_D;
    double r_d_steering_angle;
    double r_d_vs;

    double sc_quad_track;
    double sc_quad_tire;
    double sc_quad_alpha;

    double sc_lin_track;
    double sc_lin_tire;
    double sc_lin_alpha;
};

#endif //MPCC_COST_PARAMS_H
