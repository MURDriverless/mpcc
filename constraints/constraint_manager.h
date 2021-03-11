//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_CONSTRAINT_MANAGER_H
#define MPCC_CONSTRAINT_MANAGER_H

#include "../models/dynamic_bicycle_model.h"
#include "constraint_types.h"
#include "tire_ellipsis_constraint.h"
#include "tire_slip_constraint.h"
#include "track_constraint.h"

class ConstraintManager {
public:
    explicit ConstraintManager(const DynamicBicycleModel &model_args);
    ConstraintsMatrix getConstraints(const Track &track, const State &xk) const;
private:
    DynamicBicycleModel model;
};

#endif //MPCC_CONSTRAINT_MANAGER_H
