//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "mpc.h"

using Eigen::Vector2d;

MPC::MPC() :
        bounds(),
        constraints(),
        costs(),
        model(),
        params(),
        solver() {}

MPC::MPC(const BoundsManager &bounds_args, const ConstraintManager &constraint_args, const CostManager &cost_args,
         const DynamicBicycleModel &model_args, const MPCParams &params_args, const Track &track_args,
         const HpipmInterface &solver_args) {
    bounds = bounds_args;
    constraints = constraint_args;
    costs = cost_args;
    model = model_args;
    params = params_args;
    track = track_args;
    solver = solver_args;
}

OptSolution MPC::runMPC(const State &x0) {
    auto t1 = std::chrono::high_resolution_clock::now();
    int solver_status = -1;
    State xk = x0;
    xk(IndexMap.s) = track.path.projectOnSpline(x0);
    xk = mpcc::constrainState(xk, track.path.getLength());
    if (valid_initial_guess) {
        updateInitialGuess(xk);
    } else {
        generateInitialGuess(xk);
    }

    params.n_no_solves_sqp = 0;
    for (int i = 0; i < params.n_sqp; i++) {
        setProblem();
        optimal_solution = solver.solveMPC(stages, x0, &solver_status);
        if (solver_status != 0)
            params.n_no_solves_sqp++;
        if (solver_status <= 1)
            initial_guess = sqpMix(initial_guess, optimal_solution);
    }

    const int max_error = std::max(params.n_sqp-1, 1);
    if (params.n_no_solves_sqp >= max_error)
        params.n_non_solves++;
    else
        params.n_non_solves = 0;
    if (params.n_non_solves >= params.n_reset)
        valid_initial_guess = false;

    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    double time_nmpc = time_span.count();

    return { initial_guess[0].uk, initial_guess, time_nmpc };
}

void MPC::setProblem() {
    for (int i = 0; i <= N; i++) {
        setStage(initial_guess[i].xk, initial_guess[i].uk, i);
    }
}

void MPC::setStage(const State &xk, const Input &uk, int k) {
    stages[k].nx = NX;
    stages[k].nu = NU;

    // If we're at time-step k=0, which means the current "real" state we are in,
    // we have not done any predictions, and no constraints is needed as the state
    // has already happened (lol Tenet)
    if (k == 0) {
        stages[k].ng = 0;
        stages[k].ns = 0;
    } else {
        stages[k].ng = NPC;
        stages[k].ns = NS;
    }

    State xk_nz = mpcc::vxNonZero(xk, model.params.vx_zero);
    // Set 3 core elements of an optimisation problem (model is an exception actually)
    stages[k].constraints = constraints.getConstraints(track, xk);
    stages[k].costs = costs.getCosts(track.path, xk);
    stages[k].model = model.lineariseExpm(xk, uk, params.Ts);
    // Set bounds
    stages[k].x_lower = bounds.x_lower;
    stages[k].x_upper = bounds.x_upper;
    stages[k].u_lower = bounds.u_lower;
    stages[k].u_upper = bounds.u_upper;
    stages[k].s_lower = bounds.s_lower;
    stages[k].s_upper = bounds.s_upper;
}

void MPC::updateInitialGuess(const State &x0) {
    for (int i = 1; i < N; i++) {
        initial_guess[i-1] = initial_guess[i];
    }
    initial_guess[0].xk = x0;
    initial_guess[0].uk.setZero();
    initial_guess[N-1].xk = initial_guess[N-2].xk;
    initial_guess[N-1].uk.setZero();
    initial_guess[N].xk = model.predictRK4(initial_guess[N-1].xk, initial_guess[N-1].uk, params.Ts);
    initial_guess[N].uk.setZero();
    constrainInitialGuess();
}

void MPC::generateInitialGuess(const State &x0) {
    initial_guess[0].xk = x0;
    initial_guess[0].uk.setZero();
    Vector2d path_pos_i;
    Vector2d path_dpos_i;
    for (int i = 0; i <= N; i++) {
        initial_guess[i].xk.setZero();
        initial_guess[i].uk.setZero();
        initial_guess[i].xk(IndexMap.s) = initial_guess[i-1].xk(IndexMap.s) + params.Ts*model.params.initial_vx;
        path_pos_i = track.path.getPosition(initial_guess[i].xk(IndexMap.s));
        path_dpos_i = track.path.getDerivative(initial_guess[i].xk(IndexMap.s));
        initial_guess[i].xk(IndexMap.X) = path_pos_i(0);
        initial_guess[i].xk(IndexMap.Y) = path_pos_i(1);
        initial_guess[i].xk(IndexMap.yaw) = atan2(path_dpos_i(1), path_dpos_i(0));
        initial_guess[i].xk(IndexMap.vx) = model.params.initial_vx;
        initial_guess[i].xk(IndexMap.vs) = model.params.initial_vx;
    }
    constrainInitialGuess();
    valid_initial_guess = true;
}

void MPC::constrainInitialGuess() {
    double L = track.path.getLength();
    for (int i = 0; i <= N; i++) {
        if ((initial_guess[i].xk(IndexMap.yaw)-initial_guess[i-1].xk(IndexMap.yaw)) < -M_PI) {
            initial_guess[i].xk(IndexMap.yaw) += 2.0*M_PI;
        }
        if ((initial_guess[i].xk(IndexMap.yaw)-initial_guess[i-1].xk(IndexMap.yaw)) > M_PI) {
            initial_guess[i].xk(IndexMap.yaw) -= 2.0*M_PI;
        }
        if ((initial_guess[i].xk(IndexMap.s)-initial_guess[i-1].xk(IndexMap.s)) > L/2.0) {
            initial_guess[i].xk(IndexMap.s) -= L;
        }
    }
}

array<OptVariable, N + 1> MPC::sqpMix(const array<OptVariable, N + 1> &previous_sol,
                                      const array<OptVariable, N + 1> &current_sol) const {
    array<OptVariable, N+1> updated_sol;
    State updated_xk;
    Input updated_uk;
    for (int i = 0; i <= N; i++) {
        updated_xk = params.sqp_mixing * current_sol[i].xk;
        updated_uk = params.sqp_mixing * current_sol[i].uk;
        updated_sol[i].xk = updated_xk;
        updated_sol[i].uk = updated_uk;
    }
    return updated_sol;
}
