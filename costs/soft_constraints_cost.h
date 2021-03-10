//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_SOFT_CONSTRAINTS_COST_H
#define MPCC_SOFT_CONSTRAINTS_COST_H

#include "../constraints/constraint_types.h"
#include "../models/state.h"
#include "../params/cost_params.h"
#include "cost_types.h"

class SoftConstraintsCost {
public:
    SoftConstraintsCost();
    explicit SoftConstraintsCost(CostParams *cost_params);
    CostTerm<Z_MPC, z_MPC> getCost() const;
private:
    CostParams *costParams;
};

#endif //MPCC_SOFT_CONSTRAINTS_COST_H
