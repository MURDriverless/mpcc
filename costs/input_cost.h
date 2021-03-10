//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_INPUT_COST_H
#define MPCC_INPUT_COST_H

#include "../models/state.h"
#include "../params/cost_params.h"
#include "cost_types.h"

class InputCost {
public:
    InputCost();
    explicit InputCost(CostParams *cost_params);
    CostTerm<Q_MPC, q_MPC> getRawInputCost() const;
    CostTerm<R_MPC, r_MPC> getInputChangeCost() const;
private:
    CostParams *costParams;
};

#endif //MPCC_INPUT_COST_H
