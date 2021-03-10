//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_DYNAMIC_BICYCLE_H
#define MPCC_DYNAMIC_BICYCLE_H

#include <unsupported/Eigen/MatrixFunctions>
#include "../params/model_params.h"
#include "state.h"
#include "model_interface.h"

// Refers to front and rear tire forces
struct TireForces {
    // Fy (lateral force)
    const double Fy;
    // Fx (longitudinal force)
    const double Fx;
};

// Jacobian terms of tire forces. From the report, the common
// dependencies are vx, vy, wz, accel_D and steering_angle
struct TireForcesDerivatives {
    // Fy (lateral force)
    const double dFy_vx;
    const double dFy_vy;
    const double dFy_wz;
    const double dFy_accel_D;
    const double dFy_steering_angle;
    // Fx (longitudinal force)
    const double dFx_vx;
    const double dFx_vy;
    const double dFx_wz;
    const double dFx_accel_D;
    const double dFx_steering_angle;
};

class DynamicBicycleModel : public ModelInterface {
public:
    DynamicBicycleModel();
    explicit DynamicBicycleModel(const ModelParams &modelParams);
    State predictRK4(const State &xk, const Input &uk, double Ts) const override;
    LinModelMatrix lineariseExpm(const State &xk, const Input &uk, double Ts) const override;

    double getSlipAngleFront(const State &xk) const;
    double getSlipAngleRear(const State &xk) const;

    TireForces getForceFront(const State &xk) const;
    TireForcesDerivatives getForceFrontDerivatives(const State &xk) const;

    TireForces getForceRear(const State &xk) const;
    TireForcesDerivatives getForceRearDerivatives(const State &xk) const;

    ModelParams params;
private:
    State predictContinuous(const State &xk, const Input &uk) const;
    LinModelMatrix calcContinuousJacobian(const State &xk, const Input &uk) const;
};

#endif //MPCC_DYNAMIC_BICYCLE_H
