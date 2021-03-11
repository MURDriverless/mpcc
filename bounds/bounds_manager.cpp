//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "bounds_manager.h"

using json = nlohmann::json;

BoundsManager::BoundsManager(const std::string &file_path) {
    std::ifstream file_stream(file_path);
    json jsonBounds = json::parse(file_stream);

    // Lower state bounds
    x_lower(IndexMap.X) = jsonBounds["X_l"];
    x_lower(IndexMap.Y) = jsonBounds["Y_l"];
    x_lower(IndexMap.yaw) = jsonBounds["yaw_l"];
    x_lower(IndexMap.vx) = jsonBounds["vx_l"];
    x_lower(IndexMap.vy) = jsonBounds["vy_l"];
    x_lower(IndexMap.wz) = jsonBounds["wz_l"];
    x_lower(IndexMap.s) = jsonBounds["s_l"];
    x_lower(IndexMap.accel_D) = jsonBounds["accel_D_l"];
    x_lower(IndexMap.steering_angle) = jsonBounds["steering_angle_l"];
    x_lower(IndexMap.vs) = jsonBounds["vs_l"];

    // Upper state bounds
    x_upper(IndexMap.X) = jsonBounds["X_u"];
    x_upper(IndexMap.Y) = jsonBounds["Y_u"];
    x_upper(IndexMap.yaw) = jsonBounds["yaw_u"];
    x_upper(IndexMap.vx) = jsonBounds["vx_u"];
    x_upper(IndexMap.vy) = jsonBounds["vy_u"];
    x_upper(IndexMap.wz) = jsonBounds["wz_u"];
    x_upper(IndexMap.s) = jsonBounds["s_u"];
    x_upper(IndexMap.accel_D) = jsonBounds["accel_D_u"];
    x_upper(IndexMap.steering_angle) = jsonBounds["steering_angle_u"];
    x_upper(IndexMap.vs) = jsonBounds["vs_u"];

    // Lower input bounds
    u_lower(IndexMap.d_accel_D) = jsonBounds["d_accel_D_l"];
    u_lower(IndexMap.d_steering_angle) = jsonBounds["d_steering_angle_l"];
    u_lower(IndexMap.d_vs) = jsonBounds["d_vs_l"];

    // Upper input bounds
    u_upper(IndexMap.d_accel_D) = jsonBounds["d_accel_D_u"];
    u_upper(IndexMap.d_steering_angle) = jsonBounds["d_steering_angle_u"];
    u_upper(IndexMap.d_vs) = jsonBounds["d_vs_u"];

    // Lower soft bounds
    s_lower.setZero();

    // Upper soft bounds
    s_upper.setZero();
}
