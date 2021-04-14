//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "track.h"

using json = nlohmann::json;
using std::vector;

Track mpcc::getTrack(const std::string &file_path) {
    std::ifstream file_stream(file_path);
    json jsonTrack = json::parse(file_stream);

    vector<double> x_outer = jsonTrack["X_o"];
    vector<double> y_outer = jsonTrack["Y_o"];
    vector<double> x_inner = jsonTrack["X_i"];
    vector<double> y_inner = jsonTrack["Y_i"];
    vector<double> x_centre = jsonTrack["X"];
    vector<double> y_centre = jsonTrack["Y"];

    VectorXd X_outer = Eigen::Map<VectorXd>(x_outer.data(), x_outer.size());
    VectorXd Y_outer = Eigen::Map<VectorXd>(y_outer.data(), y_outer.size());
    VectorXd X_inner = Eigen::Map<VectorXd>(x_inner.data(), x_inner.size());
    VectorXd Y_inner = Eigen::Map<VectorXd>(y_inner.data(), y_inner.size());
    VectorXd X_centre = Eigen::Map<VectorXd>(x_centre.data(), x_centre.size());
    VectorXd Y_centre = Eigen::Map<VectorXd>(y_centre.data(), y_centre.size());

    CubicSpline2D outer = CubicSpline2D(X_outer, Y_outer, 3.0);
    CubicSpline2D inner = CubicSpline2D(X_inner, Y_inner, 3.0);
    CubicSpline2D centre = CubicSpline2D(X_centre, Y_centre, 3.0);
    CubicSpline2D path = CubicSpline2D(X_centre, Y_centre, 3.0);

    return { outer, inner, centre, path };
}
