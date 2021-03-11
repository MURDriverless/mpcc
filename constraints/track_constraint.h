//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_TRACK_CONSTRAINT_H
#define MPCC_TRACK_CONSTRAINT_H

#include "../splines/track.h"
#include "../constraints/constraint_types.h"

class TrackConstraint {
public:
    // Default is 0.1 (10%): car can only use up to 90% of relative track width
    static Constraint1D getTrackConstraint(const Track &track, const State &xk, double safety_margin = 0.1);
};

#endif //MPCC_TRACK_CONSTRAINT_H
