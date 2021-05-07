//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#ifndef MPCC_TRACK_H
#define MPCC_TRACK_H

#include <fstream>
#include <string>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <vector>
#include "cubic_spline2d.h"

struct Track {
    CubicSpline2D outer;
    CubicSpline2D inner;
    CubicSpline2D centre;
    CubicSpline2D path;
};

namespace mpcc {
    Track getTrack(const std::string &file_path);
    Track plannerTrack(std::vector<double> x_outer, std::vector<double> y_outer, 
                    std::vector<double> x_inner, std::vector<double> y_inner, 
                    std::vector<double> x_centre, std::vector<double> y_centre);
}

#endif //MPCC_TRACK_H
