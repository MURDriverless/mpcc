//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "cost_params.h"

using json = nlohmann::json;

CostParams::CostParams(const std::string &file_path) {
    std::ifstream file_stream(file_path);
    json jsonCost = json::parse(file_stream);

    // Assign to variables
    q_c = jsonCost["q_c"];
    q_l = jsonCost["q_l"];
    q_vs = jsonCost["q_vs"];
    q_beta = jsonCost["q_beta"];
    r_accel_D = jsonCost["r_accel_D"];
    r_steering_angle = jsonCost["r_steering_angle"];
    r_vs = jsonCost["r_vs"];
    r_d_accel_D = jsonCost["r_d_accel_D"];
    r_d_steering_angle = jsonCost["r_d_steering_angle"];
    r_d_vs = jsonCost["r_d_vs"];
    sc_quad_track = jsonCost["sc_quad_track"];
    sc_quad_tire = jsonCost["sc_quad_tire"];
    sc_quad_alpha = jsonCost["sc_quad_alpha"];
    sc_lin_track = jsonCost["sc_lin_track"];
    sc_lin_tire = jsonCost["sc_lin_tire"];
    sc_lin_alpha = jsonCost["sc_lin_alpha"];
}
