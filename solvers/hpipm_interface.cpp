//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "hpipm_interface.h"

void HpipmInterface::setDynamics(array<OptStage, N + 1> &stages, const State &x0) {
    b0 = (stages[0].model.A*x0 + stages[0].model.g);
    for (int i = 0; i < N; i++) {
        if (i == 0) {
            hA[i] = nullptr;
            hB[i] = stages[i].model.B.data();
            hb[i] = b0.data();
            nx[i] = 0;
            nu[i] = NU;
        } else {
            hA[i] = stages[i].model.A.data();
            hB[i] = stages[i].model.B.data();
            hb[i] = stages[i].model.g.data();
        }
    }
    nx[N] = NX;
    nu[N] = 0;
}

void HpipmInterface::setCost(array<OptStage, N + 1> &stages) {
    for (int i = 0; i <= N; i++) {
        hQ[i] = stages[i].costs.Q.data();
        hR[i] = stages[i].costs.R.data();
        hS[i] = stages[i].costs.S.data();

        hq[i] = stages[i].costs.q.data();
        hr[i] = stages[i].costs.r.data();

        if (stages[i].ns != 0) {
            hZl[i] = stages[i].costs.Z.data();
            hZu[i] = stages[i].costs.Z.data();
            hzl[i] = stages[i].costs.z.data();
            hzu[i] = stages[i].costs.z.data();
        } else {
            hZl[i] = nullptr;
            hZu[i] = nullptr;
            hzl[i] = nullptr;
            hzu[i] = nullptr;
        }
    }
}

void HpipmInterface::setBounds(array<OptStage, N + 1> &stages, const State &x0) {
    nbu[0] = 0;
    hpipm_bounds[0].idx_u.resize(0);
    hpipm_bounds[0].u_lower.resize(0);
    hpipm_bounds[0].u_upper.resize(0);
    for (int j = 0; j < NU; j++) {
        if (stages[0].u_lower(j) > -INF && stages[0].u_upper(j) < INF) {
            nbu[0]++;
            hpipm_bounds[0].idx_u.push_back(j);
            hpipm_bounds[0].u_lower.push_back(stages[0].u_lower(j));
            hpipm_bounds[0].u_upper.push_back(stages[0].u_upper(j));
        }
    }
    nbx[0] = 0;
    hbxid[0] = nullptr;
    hbuid[0] = hpipm_bounds[0].idx_u.data();

    hbxl[0] = nullptr;
    hbxu[0] = nullptr;
    hbul[0] = hpipm_bounds[0].u_lower.data();
    hbuu[0] = hpipm_bounds[0].u_upper.data();

    for (int i = 1; i <= N; i++) {
        hpipm_bounds[i].idx_u.resize(0);
        hpipm_bounds[i].u_lower.resize(0);
        hpipm_bounds[i].u_upper.resize(0);
        nbu[i] = 0;
        for (int j = 0; j < NU; j++) {
            if (stages[i].u_lower(j) > -INF && stages[i].u_upper(j) < INF) {
                nbu[i]++;
                hpipm_bounds[i].idx_u.push_back(j);
                hpipm_bounds[i].u_lower.push_back(stages[i].u_lower(j));
                hpipm_bounds[i].u_upper.push_back(stages[i].u_upper(j));
            }
        }

        hpipm_bounds[i].idx_x.resize(0);
        hpipm_bounds[i].u_lower.resize(0);
        hpipm_bounds[i].u_upper.resize(0);
        nbx[i] = 0;
        for (int j = 0; j < NX; j++) {
            if (stages[i].x_lower(j) > -INF && stages[i].x_upper(j) < INF) {
                nbx[i]++;
                hpipm_bounds[i].idx_x.push_back(j);
                hpipm_bounds[i].x_lower.push_back(stages[i].x_lower(j));
                hpipm_bounds[i].x_upper.push_back(stages[i].x_upper(j));
            }
        }

        hbxid[i] = hpipm_bounds[i].idx_x.data();
        hbuid[i] = hpipm_bounds[i].idx_x.data();
        hbxl[i] = hpipm_bounds[i].x_lower.data();
        hbul[i] = hpipm_bounds[i].x_upper.data();
        hbul[i] = hpipm_bounds[i].u_lower.data();
        hbuu[i] = hpipm_bounds[i].u_upper.data();
    }

    nbu[N] = 0;
    hbxid[N] = nullptr;
    hbul[N] = nullptr;
    hbuu[N] = nullptr;
}

void HpipmInterface::setPolytopicConstraints(array<OptStage, N + 1> &stages) {
    for (int i = 0; i <= N; i++) {
        ng[i] = stages[i].ng;
        if (stages[i].ng > 0) {
            hC[i] = stages[i].constraints.C.data();
            hD[i] = stages[i].constraints.D.data();
            hgl[i] = stages[i].constraints.dl.data();
            hgu[i] = stages[i].constraints.du.data();
        } else {
            hC[i] = nullptr;
            hD[i] = nullptr;
            hgl[i] = nullptr;
            hgu[i] = nullptr;
        }
    }
}

void HpipmInterface::setSoftConstraints(array<OptStage, N + 1> &stages) {
    for (int i = 0; i <= N; i++) {
        hpipm_bounds[i].idx_s.resize(0);
        if (stages[i].ns != 0) {
            nsbx[i] = 0;
            nsbu[i] = 0;
            nsg[i] = stages[i].ns;

            for (int j = 0; j < stages[i].ns; j++) {
                hpipm_bounds[i].idx_s.push_back(j + nbx[i] + nbu[i]);
            }

            hsid[i] = hpipm_bounds[i].idx_s.data();
            hlls[i] = stages[i].s_lower.data();
            hlus[i] = stages[i].s_upper.data();
        } else {
            nsbx[i] = 0;
            nsbu[i] = 0;
            nsg[i] = 0;
            hsid[i] = nullptr;
            hlls[i] = nullptr;
            hlus[i] = nullptr;
        }
    }
}

array<OptVariable, N + 1> HpipmInterface::solveMPC(array<OptStage, N + 1> &stages, const State &x0, int *status) {
    setDynamics(stages, x0);
    setCost(stages);
    setBounds(stages, x0);
    setPolytopicConstraints(stages);
    setSoftConstraints(stages);

    array<OptVariable, N+1> solution = solve(status);
    solution[0].xk = x0;

    return solution;
}

array<OptVariable, N + 1> HpipmInterface::solve(int *status) {
    // ocp qp dim
    hpipm_size_t dim_size = d_ocp_qp_dim_memsize(N);
    void *dim_mem = malloc(dim_size);

    struct d_ocp_qp_dim dim;
    d_ocp_qp_dim_create(N, &dim, dim_mem);

    d_ocp_qp_dim_set_all(nx, nu, nbx, nbu, ng, nsbx, nsbu, nsg, &dim);
    // ocp qp
    hpipm_size_t qp_size = d_ocp_qp_memsize(&dim);
    void *qp_mem = malloc(qp_size);

    struct d_ocp_qp qp;
    d_ocp_qp_create(&dim, &qp, qp_mem);
    d_ocp_qp_set_all(hA, hB, hb, hQ, hS, hR, hq, hr,
                     hbxid, hbxl, hbxu, hbuid, hbul, hbuu,
                     hC, hD, hgl, hgu, hZl, hZu, hzl, hzu,
                     hsid, hlls, hlus, &qp);

    // ocp qp sol
    hpipm_size_t qp_sol_size = d_ocp_qp_sol_memsize(&dim);
    void *qp_sol_mem = malloc(qp_sol_size);

    struct d_ocp_qp_sol qp_sol;
    d_ocp_qp_sol_create(&dim, &qp_sol, qp_sol_mem);

    hpipm_size_t ipm_arg_size = d_ocp_qp_ipm_arg_memsize(&dim);
    printf("\nipm arg size = %d\n", ipm_arg_size);
    void *ipm_arg_mem = malloc(ipm_arg_size);

    struct d_ocp_qp_ipm_arg arg;
    d_ocp_qp_ipm_arg_create(&dim, &arg, ipm_arg_mem);

//    enum hpipm_mode mode = SPEED_ABS;
    enum hpipm_mode mode = SPEED;
//    enum hpipm_mode mode = BALANCE;
//    enum hpipm_mode mode = ROBUST;

//    int mode = 1;
    double mu0 = 1e2;
    int iter_max = 20;
    double tol_stat = 1e-6;
    double tol_eq = 1e-6;
    double tol_ineq = 1e-6;
    double tol_comp = 1e-6;
    double reg_prim = 1e-12;
    int warm_start = 0;
    int pred_corr = 1;
    int ric_alg = 0;

    d_ocp_qp_ipm_arg_set_default(mode, &arg);

    // d_ocp_qp_ipm_arg_set_mu0(&mu0, &arg);
    d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &arg);
//    d_ocp_qp_ipm_arg_set_tol_stat(&tol_stat, &arg);
//    d_ocp_qp_ipm_arg_set_tol_eq(&tol_eq, &arg);
//    d_ocp_qp_ipm_arg_set_tol_ineq(&tol_ineq, &arg);
//    d_ocp_qp_ipm_arg_set_tol_comp(&tol_comp, &arg);
//    d_ocp_qp_ipm_arg_set_reg_prim(&reg_prim, &arg);
//    d_ocp_qp_ipm_arg_set_warm_start(&warm_start, &arg);
//    d_ocp_qp_ipm_arg_set_pred_corr(&pred_corr, &arg);
//    d_ocp_qp_ipm_arg_set_ric_alg(&ric_alg, &arg);

    hpipm_size_t ipm_size = d_ocp_qp_ipm_ws_memsize(&dim, &arg);
//    printf("\nipm size = %d\n", ipm_size);
    void *ipm_mem = malloc(ipm_size);

    struct d_ocp_qp_ipm_ws workspace;
    d_ocp_qp_ipm_ws_create(&dim, &arg, &workspace, ipm_mem);

    int hpipm_return; // 0 normal; 1 max iter; 2 linesearch issues?

    struct timeval tv0, tv1;

    gettimeofday(&tv0, nullptr); // start
    d_ocp_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
    d_ocp_qp_ipm_get_status(&workspace, &hpipm_return);
    gettimeofday(&tv1, nullptr); // stop
    double time_ocp_ipm = (tv1.tv_usec-tv0.tv_usec)/(1e6);

    printf("comp time = %f\n", time_ocp_ipm);
    printf("exitflag %d\n", hpipm_return);
    printf("ipm iter = %d\n", workspace.iter);

    // extract and print solution
    int ii;
    int nu_max = nu[0];
    for(ii=1; ii<=N; ii++)
        if(nu[ii]>nu_max)
            nu_max = nu[ii];
    double *u = (double*)malloc(nu_max*sizeof(double));
    for(ii=0; ii<=N; ii++) {
        d_ocp_qp_sol_get_u(ii, &qp_sol, u);
//        d_print_mat(1, nu_[ii], u, 1);
    }

    int nx_max = nx[0];
    for(ii=1; ii<=N; ii++)
        if(nx[ii]>nx_max)
            nx_max = nx[ii];
    double *x = (double*)malloc(nx_max*sizeof(double));
//    printf("\nx = \n");
    for(ii=0; ii<=N; ii++)
    {
        d_ocp_qp_sol_get_x(ii, &qp_sol, x);
//        d_print_mat(1, nx_[ii], x, 1);
    }

    array<OptVariable, N+1> optimal_solution;
    optimal_solution[0].xk.setZero();
    for(int i=1;i<=N;i++){
        d_ocp_qp_sol_get_x(i, &qp_sol, x);
        optimal_solution[i].xk(IndexMap.X) = x[IndexMap.X];
        optimal_solution[i].xk(IndexMap.Y) = x[IndexMap.Y];
        optimal_solution[i].xk(IndexMap.yaw) = x[IndexMap.yaw];
        optimal_solution[i].xk(IndexMap.vx) = x[IndexMap.vx];
        optimal_solution[i].xk(IndexMap.vy) = x[IndexMap.vy];
        optimal_solution[i].xk(IndexMap.wz) = x[IndexMap.wz];
        optimal_solution[i].xk(IndexMap.s) = x[IndexMap.s];
        optimal_solution[i].xk(IndexMap.accel_D) = x[IndexMap.accel_D];
        optimal_solution[i].xk(IndexMap.steering_angle) = x[IndexMap.steering_angle];
        optimal_solution[i].xk(IndexMap.vs) = x[IndexMap.vs];
    }

    for(int i=0;i<N;i++){
        d_ocp_qp_sol_get_u(i, &qp_sol, u);
        optimal_solution[i].uk(IndexMap.d_accel_D) = u[IndexMap.d_accel_D];
        optimal_solution[i].uk(IndexMap.d_steering_angle) = u[IndexMap.d_steering_angle];
        optimal_solution[i].uk(IndexMap.d_vs) = u[IndexMap.d_vs];
    }
    optimal_solution[N].uk.setZero();

    free(dim_mem);
    free(qp_mem);
    free(qp_sol_mem);
    free(ipm_arg_mem);
    free(ipm_mem);

    free(u);
    free(x);

    *status = hpipm_return;

    return optimal_solution;
}
