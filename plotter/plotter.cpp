//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "plotter.h"

Plotter::Plotter(double Ts, const ModelParams modelParams) {
    model_params = modelParams;
}

void Plotter::plot_simulation(const std::list<OptSolution> &log, const Track &track) const {
    std::vector<double> plot_xc(track.centre.sx.ft_spline.data(), track.centre.sx.ft_spline.data() + track.centre.sx.ft_spline.size());
    std::vector<double> plot_yc(track.centre.sy.ft_spline.data(), track.centre.sy.ft_spline.data() + track.centre.sy.ft_spline.size());

    std::vector<double> plot_xi(track.inner.sx.ft_spline.data(), track.inner.sx.ft_spline.data() + track.inner.sx.ft_spline.size());
    std::vector<double> plot_yi(track.inner.sy.ft_spline.data(), track.inner.sy.ft_spline.data() + track.inner.sy.ft_spline.size());
    std::vector<double> plot_xo(track.outer.sx.ft_spline.data(), track.outer.sx.ft_spline.data() + track.outer.sx.ft_spline.size());
    std::vector<double> plot_yo(track.outer.sy.ft_spline.data(), track.outer.sy.ft_spline.data() + track.outer.sy.ft_spline.size());


    std::vector<double> plot_x;
    std::vector<double> plot_y;

    for(const OptSolution& log_i : log)
    {
        plot_x.resize(0);
        plot_y.resize(0);
        for(const auto & opt_var : log_i.opt_vars)
        {
            plot_x.push_back(opt_var.xk(IndexMap.X));
            plot_y.push_back(opt_var.xk(IndexMap.Y));
        }
        plt::clf();
        plt::plot(plot_xc,plot_yc,"r--");
        plt::plot(plot_xi,plot_yi,"k-");
        plt::plot(plot_xo,plot_yo,"k-");
        plot_box(log_i.opt_vars[0].xk);
        plt::plot(plot_x,plot_y,"b-");
        plt::axis("equal");
        // plt::xlim(-2,2);
        // plt::ylim(-2,2);
        plt::pause(0.01);
    }
}

void Plotter::plot_box(const State &x0) const {
    std::vector<double> corner_x;
    std::vector<double> corner_y;
    double body_xl = std::cos(x0(IndexMap.yaw))*model_params.car_l;
    double body_xw = std::sin(x0(IndexMap.yaw))*model_params.car_w;
    double body_yl = std::sin(x0(IndexMap.yaw))*model_params.car_l;
    double body_yw = -std::cos(x0(IndexMap.yaw))*model_params.car_w;

    corner_x.push_back(x0(IndexMap.X) + body_xl + body_xw);
    corner_x.push_back(x0(IndexMap.X) + body_xl - body_xw);
    corner_x.push_back(x0(IndexMap.X) - body_xl - body_xw);
    corner_x.push_back(x0(IndexMap.X) - body_xl + body_xw);
    corner_x.push_back(x0(IndexMap.X) + body_xl + body_xw);

    corner_y.push_back(x0(IndexMap.Y) + body_yl + body_yw);
    corner_y.push_back(x0(IndexMap.Y) + body_yl - body_yw);
    corner_y.push_back(x0(IndexMap.Y) - body_yl - body_yw);
    corner_y.push_back(x0(IndexMap.Y) - body_yl + body_yw);
    corner_y.push_back(x0(IndexMap.Y) + body_yl + body_yw);

    plt::plot(corner_x,corner_y,"k-");
}
