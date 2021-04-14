//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_HPIPM_INTERFACE_H
#define MPCC_HPIPM_INTERFACE_H

#include <stdlib.h>
#include <iostream>
#include <array>
#include <sys/time.h>
#include <vector>
#include <Eigen/Dense>

#include <blasfeo_d_aux_ext_dep.h>

#include "hpipm_d_ocp_qp_ipm.h"
#include "hpipm_d_ocp_qp_dim.h"
#include "hpipm_d_ocp_qp.h"
#include "hpipm_d_ocp_qp_sol.h"
#include "hpipm_timing.h"

#include "../models/state.h"
#include "../mpc/mpc_types.h"

using std::array;
using std::vector;
using Eigen::Matrix;

struct HpipmBounds {
    vector<int> idx_u;
    vector<int> idx_x;
    vector<int> idx_s;
    vector<double> u_lower;
    vector<double> u_upper;
    vector<double> x_lower;
    vector<double> x_upper;
};

class HpipmInterface {
public:
    array<OptVariable, N+1> solveMPC(array<OptStage, N+1> &stages, const State &x0, int *status);
private:
    int nx[N+1];    // number of states
    int nu[N+1];    // number of inputs
    int nbx[N+1];   // number of state bounds
    int nbu[N+1];   // number of input bounds
    int ng[N+1];    // number of polytopic constraints
    int nsbx[N+1];  // number of slack variables on state
    int nsbu[N+1];  // number of slack variables on input
    int nsg[N+1];   // number of slack variables on polytopic constraints

    // LTV dynamics
    // xk1 = Ak*xk + Bk*uk + bk
    double *hA[N];  // hA[k] = Ak
    double *hB[N];  // hB[k] = Bk
    double *hb[N];  // hb[k] = bk

    // Cost (without soft constraints)
    // min(x,u) sum(0<=k<=N) 0.5 * [xk;uk]^T * [Qk, Sk; Sk^T, Rk] * [xk;uk] + [qk; rk]^T * [xk;uk]
    double *hQ[N+1];
    double *hS[N+1];
    double *hR[N+1];
    double *hq[N+1];
    double *hr[N+1];

    // Polytopic constraints
    // g_(lower, k) <= Dk*xk + Ck*uk <= g_(upper, k)
    double *hgl[N+1];
    double *hgu[N+1];
    double *hD[N+1];
    double *hC[N+1];

    // General bounds
    // x_(lower, k) <= xk <= x_(upper, k)
    // hbxid can be used to select bounds on a subset of states
    int *hbxid[N+1];
    double *hbxl[N+1];
    double *hbxu[N+1];
    // u_(lower, k) <= uk <= u_(upper, k)
    int *hbuid[N+1];
    double *hbul[N+1];
    double *hbuu[N+1];

    // Cost (only soft constraints)
    // s_(lower, k) -> slack variable of lower polytopic constraint (3) + lower bounds
    // s_(upper, k) -> slack variable of upper polytopic constraint (4) + upper bounds
    // min(x,u) sum(0<=k<=N) 0.5 * [s_lower, k; s_upper, k]^T * [Z_lower, k, 0; 0, Z_upper, k] * [s_lower, k; s_upper, k]
    //                      + [z_lower, k; z_upper, k]^T * [s_lower, k; s_upper, k]
    double *hZl[N+1];
    double *hZu[N+1];
    double *hzl[N+1];
    double *hzu[N+1];

    // Bounds of soft constraint multipliers
    double *hlls[N+1];
    double *hlus[N+1];
    // index of the bounds and constraints that are softened
    // order is not really clear
    int *hsid[N+1];

    // Bounds that are different to stage bounds and need to be stored somewhere
    array<HpipmBounds, N+1> hpipm_bounds;
    Matrix<double, NX, 1> b0;

    void setDynamics(array<OptStage, N+1> &stages, const State &x0);
    void setCost(array<OptStage, N+1> &stages);
    void setBounds(array<OptStage, N+1> &stages, const State &x0);
    void setPolytopicConstraints(array<OptStage, N+1> &stages);
    void setSoftConstraints(array<OptStage, N+1> &stages);

    array<OptVariable, N+1> solve(int *status);
};

#endif //MPCC_HPIPM_INTERFACE_H
