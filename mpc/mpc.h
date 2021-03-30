//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_MPC_H
#define MPCC_MPC_H

#include <chrono>
#include <cmath>
#include "../bounds/bounds_manager.h"
#include "../constraints/constraint_manager.h"
#include "../costs/cost_manager.h"
#include "../models/dynamic_bicycle_model.h"
#include "../models/state.h"
#include "../splines/track.h"
#include "mpc_params.h"
#include "mpc_types.h"
#include "../solvers/hpipm_interface.h"

using std::array;

class MPC {
public:
    MPC();
    MPC(const BoundsManager &bounds_args,
        const ConstraintManager &constraint_args,
        const CostManager &cost_args,
        const DynamicBicycleModel &model_args,
        const MPCParams &params_args,
        const Track &track_args,
        const HpipmInterface &solver_args);
    OptSolution runMPC(const State &x0);
private:
    void setProblem();
    void setStage(const State &xk, const Input &uk, int k);

    void updateInitialGuess(const State &x0);
    void generateInitialGuess(const State &x0);
    void constrainInitialGuess();
    array<OptVariable, N+1> sqpMix(const array<OptVariable, N+1> &previous_sol,
                                   const array<OptVariable, N+1> &current_sol) const;

    bool valid_initial_guess = false;
    array<OptStage, N+1> stages;
    array<OptVariable, N+1> initial_guess;
    array<OptVariable, N+1> optimal_solution;

    BoundsManager bounds;
    ConstraintManager constraints;
    CostManager costs;
    DynamicBicycleModel model;
    MPCParams params;
    Track track;

    HpipmInterface solver;
};

#endif //MPCC_MPC_H
