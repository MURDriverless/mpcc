//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_TIRE_SLIP_CONSTRAINT_H
#define MPCC_TIRE_SLIP_CONSTRAINT_H

#include "../models/dynamic_bicycle.h"
#include "../models/state.h"
#include "constraint_types.h"

class TireSlipConstraint {
public:
    // Tire slip angle constraint
    static Constraint1D getRearAlphaConstraint(const DynamicBicycleModel &model,const State &xk);
    static Constraint1D getFrontAlphaConstraint(const DynamicBicycleModel &model, const State &xk);
};

#endif //MPCC_TIRE_SLIP_CONSTRAINT_H
