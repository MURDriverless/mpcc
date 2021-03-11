//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_CONSTRAINT_TYPES_H
#define MPCC_CONSTRAINT_TYPES_H

struct ConstraintIndexStruct {
    int track = 0;  // constrain the car within the track
    int tire_rear = 1;   // rear tire force ellipsis
    int tire_front = 2;  // front tire force ellipsis
    int alpha_rear = 3;  // rear tire slip angle
    int alpha_front = 4; // front tire slip angle
};

static const ConstraintIndexStruct ConstraintIndex;

#endif //MPCC_CONSTRAINT_TYPES_H
