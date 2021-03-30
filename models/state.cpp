//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "state.h"

State mpcc::vxNonZero(const State &xk, double min_vx) {
    if (xk(IndexMap.vx) < min_vx) {
        State xk_nz = xk;
        xk_nz(IndexMap.vx) = min_vx;
        xk_nz(IndexMap.vy) = 0.0;
        xk_nz(IndexMap.wz) = 0.0;
        xk_nz(IndexMap.steering_angle) = 0.0;
        return xk_nz;
    }
    return xk;
}

State mpcc::constrainState(const State &xk, double path_length) {
    State xk_cnstr = xk;
    if (xk_cnstr(IndexMap.yaw) > M_PI) {
        xk_cnstr(IndexMap.yaw) -= 2.0*M_PI;
    }
    if (xk_cnstr(IndexMap.yaw) < -M_PI) {
        xk_cnstr(IndexMap.yaw) += 2.0*M_PI;
    }
    if (xk_cnstr(IndexMap.s) > path_length) {
        xk_cnstr(IndexMap.s) -= path_length;
    }
    if (xk_cnstr(IndexMap.s) < 0) {
        xk_cnstr(IndexMap.s) += path_length;
    }
    return xk_cnstr;
}
