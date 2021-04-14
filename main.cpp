#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

#include "costs/cost_params.h"
#include "models/model_params.h"
#include "mpc/mpc_params.h"

#include "models/dynamic_bicycle_model.h"
#include "models/state.h"

#include "bounds/bounds_manager.h"
#include "constraints/constraint_manager.h"
#include "costs/cost_manager.h"
#include "mpc/mpc.h"

#include "splines/cubic_spline2d.h"
#include "splines/track.h"

#include "solvers/hpipm_interface.h"

using std::string;
using json = nlohmann::json;
using std::cout;
using std::endl;

int main() {
    // Define params file paths
    const string bounds_params_file = "params/bounds_params.json";
    const string cost_params_file = "params/cost_params.json";
    const string model_params_file = "params/model_params.json";
    const string mpc_params_file = "params/mpc_params.json";
    const string track_file = "params/track.json";

    // Create params objects
    CostParams cost_params = CostParams(cost_params_file);
    ModelParams model_params = ModelParams(model_params_file);
    MPCParams mpc_params = MPCParams(mpc_params_file);

    // Create model
    DynamicBicycleModel model = DynamicBicycleModel(model_params);

    // Create managers
    BoundsManager bounds = BoundsManager(bounds_params_file);
    ConstraintManager constraints = ConstraintManager(model);
    CostManager costs = CostManager(cost_params, model_params);

    // Create track
    Track track = mpcc::getTrack(track_file);

    // Create solver
    HpipmInterface solver = HpipmInterface();

    // Create MPC object
    MPC mpc = MPC(bounds, constraints, costs, model, mpc_params, track, solver);

    // Construct initial state
    const Vector2d init_pos = track.path.getPosition(0);
    const double init_yaw = atan2(init_pos(1), init_pos(0));
    const double v0 = 5.0;
    State x0 = {
        init_pos(0),
        init_pos(1),
        init_yaw,
        v0,
        0.0,  // vy
        0.0,  // wz
        0.0,  // s
        0.0,  // accel_D
        0.0,  // steering_angle
        v0
    };

    // Logger object
    std::list<OptSolution> log;

    // Run simulation
    int n_sim = 3000;
    int i;
    for (i = 0; i < n_sim; i++) {
        OptSolution mpc_sol = mpc.runMPC(x0);
        x0 = model.predictRK4(x0, mpc_sol.u0, mpc_params.Ts);
        log.push_back(mpc_sol);
    }

    // Analyse execution time
    double mean_time = 0.0;
    double max_time = 0.0;
    for (OptSolution log_i : log) {
        mean_time += log_i.exec_time;
        if (log_i.exec_time > max_time) {
            max_time = log_i.exec_time;
        }
    }
    mean_time = mean_time / (double)(n_sim);
    cout << "Mean MPCC time: " << mean_time << endl;
    cout << "Max MPCC time: " << max_time << endl;
    cout << "i: " << i << endl;

    return 0;
}
