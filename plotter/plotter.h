//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_PLOTTER_H
#define MPCC_PLOTTER_H

#include <cmath>
#include <list>
#include <matplotlibcpp.h>
#include <vector>
#include "../models/model_params.h"
#include "../models/state.h"
#include "../mpc/mpc_types.h"
#include "../splines/track.h"

namespace plt = matplotlibcpp;

class Plotter {
public:
    void plot_simulation(const std::list<OptSolution> &log, const Track &track) const;
    void plot_box(const State &x0) const;
    Plotter(double Ts, ModelParams modelParams);
    ModelParams model_params;
};


#endif //MPCC_PLOTTER_H
