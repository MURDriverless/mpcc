//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "track_constraint.h"

using Eigen::Matrix2d;

Constraint1D TrackConstraint::getTrackConstraint(const Track &track, const State &xk, double safety_margin) {
    const double s = xk(IndexMap.s);

    const Vector2d outer_pos = track.outer.getPosition(s);
    const Vector2d centre_pos = track.centre.getPosition(s);

    // diff_vec = [x_diff; y_diff]
    const Vector2d diff_vec = outer_pos - centre_pos;
    const double diff_vec_norm_gain = 1 / (pow(diff_vec(0), 2) + pow(diff_vec(1), 2));

    // Container to fix the inability to use Matrix2d constructor like: Matrix2d { el(0), el(1), el(2), el(3) }
    Matrix2d T_matrix;
    T_matrix << diff_vec(0), diff_vec(1),
            -diff_vec(1), diff_vec(0);

    // G
    const Matrix<double, 2, 2> G = diff_vec_norm_gain * T_matrix;

    // Construct Jacobian first
    Matrix<double, 2, NX> J_p = Matrix<double, 2, NX>::Zero();
    // dp_dx
    J_p.col(IndexMap.X) = G * Vector2d {1.0, 0.0};
    // dp_dy
    J_p.col(IndexMap.Y) = G * Vector2d { 0.0, 1.0 };

    // dG_dtheta
    const Vector2d J_centre_pos = track.centre.getDerivative(s);
    const Vector2d J_diff_vec = track.outer.getDerivative(s) - J_centre_pos;
    // Container to fix the inability to use Matrix2d constructor like: Matrix2d { el(0), el(1), el(2), el(3) }
    Matrix2d J_T_matrix;
    J_T_matrix << J_diff_vec(0), J_diff_vec(1),
            -J_diff_vec(1), J_diff_vec(0);
    const Matrix2d dG_dtheta_1 = diff_vec_norm_gain * J_T_matrix;
    const Matrix2d dG_dtheta_2 = pow(diff_vec_norm_gain, 2) *
                                 (2*diff_vec(0)*J_diff_vec(0) + 2*diff_vec(1)*J_diff_vec(1)) *
                                 T_matrix;
    const Matrix2d dG_dtheta = dG_dtheta_1 - dG_dtheta_2;

    // dp_dtheta
    const Vector2d state_from_centre = Vector2d {
            xk(IndexMap.X) - centre_pos(0),
            xk(IndexMap.Y) - centre_pos(1)
    };
    J_p.col(IndexMap.s) = -G * J_centre_pos + dG_dtheta * state_from_centre;

    // Compute limits
    const Vector2d p_bar = G * state_from_centre;
    const double cross_track_limit_abs = 1.0 - safety_margin;
    const Vector2d lower_bound = Vector2d {-cross_track_limit_abs, -INF} - p_bar;
    const Vector2d upper_bound = Vector2d {cross_track_limit_abs, INF} - p_bar;

    // Note that we only want to constrain x-prime as it is the lateral part of the track,
    // and it does not make sense to constraint the longitudinal part of the track: y-prime
    C_i_MPC J_p_x = J_p.row(0);
    const double lower_bound_x = lower_bound(0);
    const double upper_bound_x = upper_bound(1);

    return { J_p_x, lower_bound_x, upper_bound_x };
}
