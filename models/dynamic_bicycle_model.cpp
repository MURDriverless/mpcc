//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "dynamic_bicycle_model.h"

DynamicBicycleModel::DynamicBicycleModel() : params() {}

DynamicBicycleModel::DynamicBicycleModel(const ModelParams &modelParams) {
    this->params = modelParams;
}

State DynamicBicycleModel::predictRK4(const State &xk, const Input &uk, double Ts) const {
    const State k1 = predictContinuous(xk, uk);
    const State k2 = predictContinuous((xk + k1*Ts/2.0), uk);
    const State k3 = predictContinuous((xk + k2*Ts/2.0), uk);
    const State k4 = predictContinuous((xk + k3*Ts), uk);
    State xk1 = xk + Ts * (k1/6.0 + k2/3.0 + k3/3.0 + k4/6.0);
    return xk1;
}

LinModelMatrix DynamicBicycleModel::lineariseExpm(const State &xk, const Input &uk, double Ts) const {
    // We firstly calculate the Jacobians for linearisation, then we discretise it using expm
    LinModelMatrix linModel = calcContinuousJacobian(xk, uk);

    // As per the MPCC issue (https://github.com/alexliniger/MPCC/issues/9), we can define:
    //
    // \hat{A} = [A, B, g; zeros(NU+1, NX+NU+1)]
    // \hat{x} = [x; u; 1]
    //
    // such that we "lift" the state and input up into a single matrix, so
    // the problem just becomes an exponential of this single matrix
    //
    // \dot{x} = A*x + B*u + g, to
    // \dot{\hat{x}} = \hat{A}*\hat{x}
    //
    // We can then compute the matrix exponential of \dot{\hat{x}} to perform ZOH discretisation.
    Eigen::Matrix<double, NX+NU+1, NX+NU+1> A_hat = Eigen::Matrix<double, NX+NU+1, NX+NU+1>::Zero();
    A_hat.block<NX, NX>(0, 0) = linModel.A;
    A_hat.block<NX, NU>(0, NX) = linModel.B;
    A_hat.block<NX, 1>(0, NX+NU) = linModel.g;

    // Just reassign to save computation
    A_hat = A_hat * Ts;

    // Take the matrix exponential of A_hat
    const Eigen::Matrix<double, NX+NU+1, NX+NU+1> A_hat_expm = A_hat.exp();

    // Extract the result
    const A_MPC Ad = A_hat_expm.block<NX, NX>(0, 0);
    const B_MPC Bd = A_hat_expm.block<NX, NU>(0, NX);
    const g_MPC gd = A_hat_expm.block<NX, 1>(0, NX+NU);
    return { Ad, Bd, gd };
}

double DynamicBicycleModel::getSlipAngleFront(const State &xk) const {
    // alpha_f = -atan2(vy + lf*wz, vx) + steering_angle
    return -atan2(xk(IndexMap.vy) + xk(IndexMap.wz) * params.lf, xk(IndexMap.vx)) + xk(IndexMap.steering_angle);
}

double DynamicBicycleModel::getSlipAngleRear(const State &xk) const {
    // alpha_r = -atan2(vy - lr*wz, vx)
    return -atan2(xk(IndexMap.vy) - xk(IndexMap.wz) * params.lr, xk(IndexMap.vx));
}

TireForces DynamicBicycleModel::getForceFront(const State &xk) const {
    const double alpha_f = getSlipAngleFront(xk);
    // F_fy = Df * sin(Cf * atan(Bf * alpha_f))
    const double Fy = params.Df * sin(params.Cf * atan(params.Bf * alpha_f));
    // F_fx = 0.0 as we assume front wheels do not provide actuation
    const double Fx = 0.0;
    return { Fy, Fx };
}

TireForces DynamicBicycleModel::getForceRear(const State &xk) const {
    const double alpha_r = getSlipAngleRear(xk);
    // F_ry = Dr * sin(Cr * atan(Br * alpha_r))
    const double Fy = params.Dr * sin(params.Cr * atan(params.Br*alpha_r));
    // F_rx = (Cm1 - Cm2*vx)*accel_D - Cr0 - Cr2*vx^2
    const double Fx = (params.Cm1 - params.Cm2*xk(IndexMap.vx)) * xk(IndexMap.accel_D)
                      - params.Cr0 - params.Cr2*pow(xk(IndexMap.vx), 2);
    return { Fy, Fx };
}

TireForcesDerivatives DynamicBicycleModel::getForceFrontDerivatives(const State &xk) const {
    const double alpha_f = getSlipAngleFront(xk);
    const double vx = xk(IndexMap.vx);
    const double vy = xk(IndexMap.vy);
    const double wz = xk(IndexMap.wz);

    // F_fx = 0.0 as we assume front wheels do not provide actuation
    const double dFx_vx = 0.0;
    const double dFx_vy = 0.0;
    const double dFx_wz = 0.0;
    const double dFx_accel_D = 0.0;
    const double dFx_steering_angle = 0.0;
    // F_fy = Df * sin(Cf * atan(Bf * alpha_f)), where
    // alpha_f = -atan2(vy + lf*wz, vx) + steering_angle
    const double dFy_vx = (params.Bf*params.Cf*params.Df*cos(params.Cf*atan(params.Bf*alpha_f)))
                          / (1.0 + pow(params.Bf, 2)*pow(alpha_f, 2))
                          * ((params.lf*wz + vy) / (pow(params.lf*wz + vy, 2) + pow(vx, 2)));
    const double dFy_vy = (params.Bf*params.Cf*params.Df*cos(params.Cf*atan(params.Bf*alpha_f)))
                          / (1.0 + pow(params.Bf, 2)*pow(alpha_f, 2))
                          * (-vx / (pow(params.lf*wz + vy, 2) + pow(alpha_f, 2)));
    const double dFy_wz = (params.Bf*params.Cf*params.Df*cos(params.Cf*atan(params.Bf*alpha_f)))
                          / (1.0 + pow(params.Bf, 2)*pow(alpha_f, 2))
                          * ((-params.lf*vx) / (pow(params.lf*wz + vy, 2) + pow(vx, 2)));
    const double dFy_accel_D = 0.0;
    const double dFy_steering_angle = (params.Bf*params.Cf*params.Df*cos(params.Cf*atan(params.Bf*alpha_f)))
                                      / (1.0 + pow(params.Bf, 2)*pow(alpha_f, 2));
    return {
            dFy_vx, dFy_vy, dFy_wz, dFy_accel_D, dFy_steering_angle,
            dFx_vx, dFx_vy, dFx_wz, dFx_accel_D, dFx_steering_angle
    };
}

TireForcesDerivatives DynamicBicycleModel::getForceRearDerivatives(const State &xk) const {
    const double alpha_r = getSlipAngleRear(xk);
    const double vx = xk(IndexMap.vx);
    const double vy = xk(IndexMap.vy);
    const double wz = xk(IndexMap.wz);
    const double accel_D = xk(IndexMap.accel_D);

    // F_rx = (Cm1 - Cm2*vx)*accel_D - Cr0 - Cr2*vx^2
    const double dFx_vx = -params.Cm2*accel_D - 2*params.Cr2*vx;
    const double dFx_vy = 0.0;
    const double dFx_wz = 0.0;
    const double dFx_accel_D = params.Cm1 - params.Cm2*vx;
    const double dFx_steering_angle = 0.0;
    // F_ry = Dr * sin(Cr * atan(Br * alpha_r)), where
    // alpha_r = -atan2(vy - lr*wz, vx)
    const double dFy_vx = ((params.Br*params.Cr*params.Dr*cos(params.Cr*atan(params.Br*alpha_r)))
                           / (1.0 + pow(params.Br, 2)*pow(alpha_r, 2))) *
                          (-(params.lr*wz - vy) / (pow((-params.lr*wz + vy), 2) + pow(vx, 2)));
    const double dFy_vy = ((params.Br*params.Cr*params.Dr*cos(params.Cr*atan(params.Br*alpha_r)))
                           / (1.0 + pow(params.Br, 2)*pow(alpha_r, 2))) *
                          ((-vx) / (pow(-params.lr*wz + vy, 2) + pow(vx, 2)));
    const double dFy_wz = ((params.Br*params.Cr*params.Dr*cos(params.Cr*atan(params.Br*alpha_r)))
                           / (1.0 + pow(params.Br, 2)*pow(alpha_r, 2))) *
                          ((params.lr*vx) / (pow(-params.lr*wz + vy, 2) + pow(vx, 2)));
    const double dFy_accel_D = 0.0;
    const double dFy_steering_angle = 0.0;
    return {
            dFy_vx, dFy_vy, dFy_wz, dFy_accel_D, dFy_steering_angle,
            dFx_vx, dFx_vy, dFx_wz, dFx_accel_D, dFx_steering_angle
    };
}

State DynamicBicycleModel::predictContinuous(const State &xk, const Input &uk) const {
    // Destructure state
    const double yaw = xk(IndexMap.yaw);
    const double vx = xk(IndexMap.vx);
    const double vy = xk(IndexMap.vy);
    const double wz = xk(IndexMap.wz);
    const double steering_angle = xk(IndexMap.steering_angle);
    const double vs = xk(IndexMap.vs);
    // Destructure input
    const double d_accel_D = uk(IndexMap.d_accel_D);
    const double d_steering_angle = uk(IndexMap.d_steering_angle);
    const double d_vs = uk(IndexMap.d_vs);

    const TireForces front_force = getForceFront(xk);
    const TireForces rear_force = getForceRear(xk);

    // TODO: Add torque vectoring term in wz once it is implemented in mursim
//    const double wz_target =  (steering_angle * vx) / (params.lf + params.lr);
//    // Torque vectoring yaw moment, where P_TV is the proportional gain of low level TV controller
//    const double tau_TV = (wz_target - wz) * P_TV;

    State xdot;
    xdot(IndexMap.X) = vx * cos(yaw) - vy * sin(yaw);
    xdot(IndexMap.Y) = vx * sin(yaw) + vy * cos(yaw);
    xdot(IndexMap.yaw) = wz;
    xdot(IndexMap.vx) = 1.0 / params.m * (rear_force.Fx - front_force.Fy * sin(steering_angle) + params.m * vy * wz);
    xdot(IndexMap.vy) = 1.0 / params.m * (rear_force.Fy + front_force.Fy * cos(steering_angle) - params.m * vx * wz);
    xdot(IndexMap.wz) = 1.0 / params.Iz * (front_force.Fy * params.lf * cos(steering_angle) - rear_force.Fy * params.lr);
    xdot(IndexMap.s) = vs;
    xdot(IndexMap.accel_D) = d_accel_D;
    xdot(IndexMap.steering_angle) = d_steering_angle;
    xdot(IndexMap.vs) = d_vs;
    return xdot;
}

LinModelMatrix DynamicBicycleModel::calcContinuousJacobian(const State &xk, const Input &uk) const {
    // Destructure state
    const double yaw = xk(IndexMap.yaw);
    const double vx = xk(IndexMap.vx);
    const double vy = xk(IndexMap.vy);
    const double wz = xk(IndexMap.wz);
    const double steering_angle = xk(IndexMap.steering_angle);

    A_MPC Ac = A_MPC::Zero();
    B_MPC Bc = B_MPC::Zero();
    g_MPC gc = g_MPC::Zero();

    const State xdot = predictContinuous(xk, uk);

    const TireForces F_front = getForceFront(xk);
    const TireForcesDerivatives dF_front = getForceFrontDerivatives(xk);
    const TireForcesDerivatives dF_rear = getForceRearDerivatives(xk);

    // Common term, declare here to minimise repetitive division
    const double mass_inv = 1.0 / params.m;
    const double Iz_inv = 1.0 / params.Iz;

    // Derivatives of function
    // f1 = \dot{X} = vx*cos(yaw) - vy*sin(yaw)
    const double df1_dyaw = -vx*sin(yaw) - vy*cos(yaw);
    const double df1_dvx  = cos(yaw);
    const double df1_dvy  = -sin(yaw);

    // f2 = \dot{Y} = vy*cos(yaw) + vx*sin(yaw)
    const double df2_dyaw = -vy*sin(yaw) + vx*cos(yaw);
    const double df2_dvx  = sin(yaw);
    const double df2_dvy = cos(yaw);

    // f3 = \dot{yaw} = wz
    const double df3_dwz = 1.0;

    // f4 = \dot{vx} = 1/m*(F_rx - F_fy*sin(steering_angle) + m*vy*wz);
    const double df4_dvx = mass_inv * (dF_rear.dFx_vx - dF_front.dFy_vx*sin(steering_angle));
    const double df4_dvy = mass_inv * (-dF_front.dFy_vy*sin(steering_angle) + params.m*wz);
    const double df4_dwz = mass_inv * (-dF_front.dFy_wz*sin(steering_angle) + params.m*vy);
    const double df4_daccel_D = mass_inv * dF_rear.dFx_accel_D;
    const double df4_dsteering_angle = mass_inv * (-dF_front.dFy_steering_angle*sin(steering_angle) - F_front.Fy*cos(steering_angle));

    // f5 = \dot{vy} = 1/m*(F_ry + F_fy*cos(steering_angle) - m*vx*wz);
    const double df5_dvx = mass_inv * (dF_rear.dFy_vx + dF_front.dFy_vx*cos(steering_angle) - params.m*wz);
    const double df5_dvy = mass_inv * (dF_rear.dFy_vy + dF_front.dFy_vy*cos(steering_angle));
    const double df5_dwz = mass_inv * (dF_rear.dFy_wz + dF_front.dFy_wz*cos(steering_angle) - params.m*vx);
    const double df5_dsteering_angle = mass_inv * (dF_front.dFy_steering_angle*cos(steering_angle) - F_front.Fy*sin(steering_angle));

    // f6 = \dot{wz} = 1/Iz*(F_fy*lf*cos(steering_angle)- F_ry*lr)
    const double df6_dvx = Iz_inv * (dF_front.dFy_vx*params.lf*cos(steering_angle) - dF_rear.dFy_vx*params.lr);
    const double df6_dvy = Iz_inv * (dF_front.dFy_vy*params.lf*cos(steering_angle) - dF_rear.dFy_vy*params.lr);
    const double df6_dwz = Iz_inv * (dF_front.dFy_wz*params.lf*cos(steering_angle) - dF_rear.dFy_wz*params.lr);
    const double df6_dsteering_angle = Iz_inv * (dF_front.dFy_steering_angle*params.lf*cos(steering_angle) - F_front.Fy*params.lf*sin(steering_angle));

    // Jacobian of A
    // Column 1 (x): all zero
    // Column 2 (y): all zero
    // Column 3 (yaw):
    Ac(IndexMap.X, IndexMap.yaw) = df1_dyaw;
    Ac(IndexMap.Y, IndexMap.yaw) = df2_dyaw;
    // Column 4 (vx):
    Ac(IndexMap.X, IndexMap.vx) = df1_dvx;
    Ac(IndexMap.Y, IndexMap.vx) = df2_dvx;
    Ac(IndexMap.vx, IndexMap.vx) = df4_dvx;
    Ac(IndexMap.vy, IndexMap.vx) = df5_dvx;
    Ac(IndexMap.wz, IndexMap.vx) = df6_dvx;
    // Column 5 (vy):
    Ac(IndexMap.X, IndexMap.vy) = df1_dvy;
    Ac(IndexMap.Y, IndexMap.vy) = df2_dvy;
    Ac(IndexMap.vx, IndexMap.vy) = df4_dvy;
    Ac(IndexMap.vy, IndexMap.vy) = df5_dvy;
    Ac(IndexMap.wz, IndexMap.vy) = df6_dvy;
    // Column 6 (wz):
    Ac(IndexMap.yaw, IndexMap.wz) = df3_dwz;
    Ac(IndexMap.vx, IndexMap.wz) = df4_dwz;
    Ac(IndexMap.vy, IndexMap.wz) = df5_dwz;
    Ac(IndexMap.wz, IndexMap.wz) = df6_dwz;
    // Column 7 (s): all zero
    // Column 8 (accel_D):
    Ac(IndexMap.vx, IndexMap.accel_D) = df4_daccel_D;
    // Column 9 (steering angle):
    Ac(IndexMap.vx, IndexMap.steering_angle) = df4_dsteering_angle;
    Ac(IndexMap.vy, IndexMap.steering_angle) = df5_dsteering_angle;
    Ac(IndexMap.wz, IndexMap.steering_angle) = df6_dsteering_angle;
    // Column 10 (vs):
    Ac(IndexMap.s, IndexMap.vs) = 1.0;

    // Jacobian of B
    // Column 1 (d_accel_D):
    Bc(IndexMap.accel_D, IndexMap.d_accel_D) = 1.0;
    // Column 2 (d_steering_angle):
    Bc(IndexMap.steering_angle, IndexMap.d_steering_angle) = 1.0;
    // Column 3 (d_vs):
    Bc(IndexMap.vs, IndexMap.d_vs) = 1.0;

    // Zero order term
    gc = xdot - Ac*xk - Bc*uk;

    return {Ac, Bc, gc };
}
