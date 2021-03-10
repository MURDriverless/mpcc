//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_SLIP_ANGLE_COST_H
#define MPCC_SLIP_ANGLE_COST_H

#include "../models/state.h"
#include "../params/cost_params.h"
#include "../params/model_params.h"
#include "./cost_types.h"

// General container to hold linearisation variables: setpoint value (bar) and Jacobian (jac)
struct LinearisedVar {
    const double bar;
    const Matrix<double, 1, NX> jac;
};

class SlipAngleCost {
public:
    SlipAngleCost();
    SlipAngleCost(CostParams *cost_params, ModelParams *model_params);
    CostTerm<Q_MPC, q_MPC> getCost(const State &xk) const;
private:
    static LinearisedVar getBetaKin(const State &xk, double lf, double lr);
    static LinearisedVar getBetaDyn(const State &xk);
    CostTerm<Q_MPC, q_MPC> getBetaCost(const State &xk) const;

    CostParams *costParams;
    ModelParams *modelParams;
};

#endif //MPCC_SLIP_ANGLE_COST_H
