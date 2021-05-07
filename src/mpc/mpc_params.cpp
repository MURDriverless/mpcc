//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "mpc_params.h"

using json = nlohmann::json;

MPCParams::MPCParams(const std::string &file_path) {
    std::ifstream file_stream(file_path);
    json jsonConfig = json::parse(file_stream);

    n_sqp = jsonConfig["n_sqp"];
    sqp_mixing = jsonConfig["sqp_mixing"];
    n_non_solves = jsonConfig["n_non_solves"];
    n_no_solves_sqp = jsonConfig["n_no_solves_sqp"];
    n_reset = jsonConfig["n_reset"];
    Ts = jsonConfig["Ts"];
}
