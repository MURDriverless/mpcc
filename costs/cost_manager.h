//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_COST_MANAGER_H
#define MPCC_COST_MANAGER_H

#include "../constraints/constraint_types.h"
#include "../models/model_params.h"
#include "../splines/cubic_spline2d.h"
#include "cost_params.h"
#include "contouring_error.h"
#include "cost_types.h"
#include "slip_angle.h"

class CostManager {
public:
    CostManager(const CostParams &cost_params, const ModelParams &model_params);
    CostMatrix getCost(const CubicSpline2D &path, const State &xk) const;
private:
    CostTerm<Q_MPC, q_MPC> getContouringCost(const CubicSpline2D &path, const State &xk) const;
    CostTerm<Q_MPC, q_MPC> getRawInputCost() const;
    CostTerm<R_MPC, r_MPC> getInputChangeCost() const;
    CostTerm<Z_MPC, z_MPC> getSoftConstraintsCost() const;
    CostTerm<Q_MPC, q_MPC> getBetaCost(const State &xk) const;

    CostParams costParams;
    ModelParams modelParams;
};

#endif //MPCC_COST_MANAGER_H
