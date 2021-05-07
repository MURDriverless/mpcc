#include <iostream>
#include <string>

#include <External/Json/include/nlohmann/json.hpp>

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

#include "ros/ros.h" // Must include for all ROS C++
#include "ROSnode/fastlapnode.h"

using std::string;
using json = nlohmann::json;
using std::cout;
using std::endl;

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "fast_lap_control");

    // Define params file paths
    const string bounds_params_file = "/workspace/src/simulation/mpcc/src/params/bounds_params.json";
    const string cost_params_file = "/workspace/src/simulation/mpcc/src/params/cost_params.json";
    const string model_params_file = "/workspace/src/simulation/mpcc/src/params/model_params.json";
    const string mpc_params_file = "/workspace/src/simulation/mpcc/src/params/mpc_params.json";
    // const string track_file = "params/track.json";

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

    // Create solver
    HpipmInterface solver = HpipmInterface();

    // Logger object
    std::list<OptSolution> log;
    
    // Create State object for SLAM update
    State x0;

    // Create empty MPC object
    MPC mpc;

    // Create empty Track
    Track track;

    // // Construct initial state to test transition
    // const Vector2d init_pos = track.path.getPosition(0);
    // const double init_yaw = atan2(init_pos(1), init_pos(0));
    // const double v0 = 5.0;
    // State x0 = {
    //     init_pos(0),
    //     init_pos(1),
    //     init_yaw,
    //     v0,
    //     0.0,  // vy
    //     0.0,  // wz
    //     0.0,  // s
    //     0.0,  // accel_D
    //     0.0,  // steering_angle
    //     v0
    // };

    // Create Fast Lap Control Node
    FastLapControlNode controlNode = FastLapControlNode();

    // Create first run variable
    bool firstRun = true;

    // ROS INFO
    ROS_INFO_STREAM("FAST LAP CONTROL STARTED.");

    while(ros::ok())
    {
        // Update 
        ros::spinOnce();

        // Fast Lap Control is ready
        if (firstRun && controlNode.getFastLapReady())
        {
            // ROS INFO
            ROS_INFO_STREAM("FAST LAP CONTROL IS READY TO GO.");
            
            // Create Track
            track = controlNode.generateTrack();

            // Create MPC object
            mpc = MPC(bounds, constraints, costs, model, mpc_params, track, solver);

            // Initialize States from transition with SLAM
            x0 = controlNode.initialize();

            firstRun = false; // Finish initialization

            // Run MPCC
            OptSolution mpc_sol = mpc.runMPC(x0);

            // Publish commands
            controlNode.publishActuation(mpc_sol.u0(IndexMap.d_accel_D), mpc_sol.u0(IndexMap.d_steering_angle));

            // ROS INFO
            ROS_INFO("PUBLISHING COMMANDS, ACCEL: %lf, STEER: %lf\n", mpc_sol.u0(IndexMap.d_accel_D), mpc_sol.u0(IndexMap.d_steering_angle));

            // Update states
            // x0 = model.predictRK4(x0, mpc_sol.u0, mpc_params.Ts); // Independent of SLAM
            x0 = controlNode.update(x0); // Update state with SLAM data
            log.push_back(mpc_sol);
        }

        else if (controlNode.getFastLapReady())
        {
            ROS_INFO("States:\nx:%lf\ny:%lf\ns:%lf.", x0(IndexMap.X), x0(IndexMap.Y), x0(IndexMap.s));  
                   
            // Run MPCC
            OptSolution mpc_sol = mpc.runMPC(x0);

            // Publish commands
            controlNode.publishActuation(mpc_sol.u0(IndexMap.d_accel_D), mpc_sol.u0(IndexMap.d_steering_angle));

            // Update states
            // x0 = model.predictRK4(x0, mpc_sol.u0, mpc_params.Ts); // Independent of SLAM
            x0 = controlNode.update(x0); // Update state with SLAM data
            log.push_back(mpc_sol);

            ROS_INFO_STREAM("Track Length" << track.path.getLength());

            // Analyse execution time
            // double mean_time = 0.0;
            // double max_time = 0.0;
            // for (OptSolution log_i : log) {
            //     mean_time += log_i.exec_time;
            //     if (log_i.exec_time > max_time) {
            //         max_time = log_i.exec_time;
            //     }
            // }
            // mean_time = mean_time / (double)(n_sim);
            // cout << "Mean MPCC time: " << mean_time << endl;
            // cout << "Max MPCC time: " << max_time << endl;
            // cout << "i: " << i << endl;

            // TODO: Lap count
        }
    }

    return 0;
}
