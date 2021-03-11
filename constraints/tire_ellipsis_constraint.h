//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_TIRE_ELLIPSIS_CONSTRAINT_H
#define MPCC_TIRE_ELLIPSIS_CONSTRAINT_H

#include "../models/dynamic_bicycle_model.h"
#include "../models/state.h"
#include "constraint_types.h"

class TireEllipsisConstraint {
public:
    // Tire force ellipsis constraint
    static Constraint1D getRearTireConstraint(const DynamicBicycleModel &model, const State &xk);
    static Constraint1D getFrontTireConstraint(const DynamicBicycleModel &model, const State &xk);
};

#endif //MPCC_TIRE_ELLIPSIS_CONSTRAINT_H
