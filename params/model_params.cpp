//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "model_params.h"

using json = nlohmann::json;

ModelParams::ModelParams() {
    // Assign all fields to be 0.0
    Cm1 = Cm2 = Cr0 = Cr2 = Br = Cr = Dr = Bf = Cf = Df = m = Iz = lf = lr =
    car_l = car_w = g = e_long = e_eps = max_alpha = vx_zero = 0.0;
}

ModelParams::ModelParams(const std::string &file_path) {
    std::ifstream file_stream(file_path);
    json jsonModel = json::parse(file_stream);

    // Assign to variables
    Cm1 = jsonModel["Cm1"];
    Cm2 = jsonModel["Cm2"];
    Cr0 = jsonModel["Cr0"];
    Cr2 = jsonModel["Cr2"];
    Br 	= jsonModel["Br"];
    Cr 	= jsonModel["Cr"];
    Dr 	= jsonModel["Dr"];
    Bf 	= jsonModel["Bf"];
    Cf 	= jsonModel["Cf"];
    Df 	= jsonModel["Df"];
    m 	= jsonModel["m"];
    Iz 	= jsonModel["Iz"];
    lf 	= jsonModel["lf"];
    lr 	= jsonModel["lr"];
    car_l = jsonModel["car_l"];
    car_w = jsonModel["car_w"];
    g = jsonModel["g"];
    e_long = jsonModel["e_long"];
    e_eps = jsonModel["e_eps"];
    max_alpha = jsonModel["max_alpha"];
    vx_zero = jsonModel["vx_zero"];
}
