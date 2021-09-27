// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#ifndef MPCC_TRACK_H
#define MPCC_TRACK_H

#include "config.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <nlohmann/json.hpp>

namespace mpcc {
//used namespace
using json = nlohmann::json;

struct TrackPos {
    Eigen::VectorXd X;
    Eigen::VectorXd Y;

    Eigen::VectorXd X_inner;
    Eigen::VectorXd Y_inner;

    Eigen::VectorXd X_outer;
    Eigen::VectorXd Y_outer;
};

class Track {
public:
    Track();
    Track(std::string file);
    Track(std::vector<double> x_outer, std::vector<double> y_outer, 
            std::vector<double> x_inner, std::vector<double> y_inner, 
            std::vector<double> x_centre, std::vector<double> y_centre);
    TrackPos getTrack();


private:
    Eigen::VectorXd X;
    Eigen::VectorXd Y;

    Eigen::VectorXd X_inner;
    Eigen::VectorXd Y_inner;

    Eigen::VectorXd X_outer;
    Eigen::VectorXd Y_outer;
};
};

#endif //MPCC_TRACK_H
