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

#include "Tests/spline_test.h"
#include "Tests/model_integrator_test.h"
#include "Tests/constratins_test.h"
#include "Tests/cost_test.h"

#include "MPC/mpc.h"
#include "Model/integrator.h"
#include "Params/track.h"
#include "Plotting/plotting.h"

#include "ros/ros.h" // Must include for all ROS C++
#include "ROSnode/fastlapnode.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <stack>
#include <ctime>
#include <chrono>

// Time testing
std::stack<clock_t> tictoc_stack;

void tic() {
    tictoc_stack.push(clock());
}

void toc() {
    std::cout << "Time elapsed: "
              << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
              << std::endl;
    tictoc_stack.pop();
}

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "alex_mpcc");

    using namespace mpcc;
    std::ifstream iConfig("/workspace/src/simulation/alexmpcc/src/Params/config.json");
    json jsonConfig;
    iConfig >> jsonConfig;

    PathToJson json_paths {jsonConfig["model_path"],
                           jsonConfig["cost_path"],
                           jsonConfig["bounds_path"],
                           jsonConfig["track_path"],
                           jsonConfig["normalization_path"]};

    // std::cout << testSpline() << std::endl;
    // std::cout << testArcLengthSpline(json_paths) << std::endl;

    // std::cout << testIntegrator(json_paths) << std::endl;
    // std::cout << testLinModel(json_paths) << std::endl;

    // std::cout << testAlphaConstraint(json_paths) << std::endl;
    // std::cout << testTireForceConstraint(json_paths) << std::endl;
    // std::cout << testTrackConstraint(json_paths) << std::endl;

    // std::cout << testCost(json_paths) << std::endl;

    Integrator integrator = Integrator(jsonConfig["Ts"],json_paths);
    Plotting plotter = Plotting(jsonConfig["Ts"],json_paths);

    Track track = Track(json_paths.track_path);
    TrackPos track_xy = track.getTrack();

    // Create ROS Node stuff
    FastLapControlNode controlNode = FastLapControlNode();
    bool firstRun = true;

    // ROS INFO
    ROS_INFO_STREAM("FAST LAP CONTROL STARTED.");

    // Empty Objects
    MPC mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths);
    State x0;
    std::list<MPCReturn> log;

    // Set Ts
    ros::Rate rate(5); // 1/Ts, Ts = 0.1

    mpc.setTrack(track_xy.X,track_xy.Y); // TO REMOVE WHEN WITH SLOW LAP

    int count = 0;

    x0 = controlNode.initialize();

    while(ros::ok())
    {
        if (count < 20)
        {
            count++;
        }   
        else if (count > 500)
        {
            controlNode.fastlapready = false; // If want start fast lap immediately
            double mean_time = 0.0;
            double max_time = 0.0;
            for(MPCReturn log_i : log)
            {
                mean_time += log_i.time_total;
                if(log_i.time_total > max_time)
                    max_time = log_i.time_total;
            }
            // std::cout << "mean nmpc time " << mean_time/double(jsonConfig["n_sim"]) << std::endl;
            std::cout << "mean nmpc time " << mean_time/count << std::endl;
            std::cout << "max nmpc time " << max_time << std::endl;
            plotter.plotSim(log,track_xy);
            return 0;
        }
        else
        {
            count++;
            controlNode.fastlapready = true; // If want start fast lap immediately
        }
        // Update 
        ros::spinOnce();

        // ROS_INFO("States:\nx:%lf\ny:%lf\nyaw:%lf\ns:%lf.", x0.X, x0.Y, x0.phi, x0.s);  
                
        // MPCReturn mpc_sol = mpc.runMPC(x0); // Updates s
        // controlNode.publishActuation(mpc_sol.u0.dD, mpc_sol.u0.dDelta);
        // log.push_back(mpc_sol);
        // // x0 = integrator.simTimeStep(x0,mpc_sol.u0,jsonConfig["Ts"]); // Updates s and vs

        // x0 = controlNode.update(x0, mpc_sol.u0, jsonConfig["Ts"]); // Update state with SLAM data 

        // Fast Lap Control is ready
        if (firstRun && controlNode.getFastLapReady())
        {
            // ROS INFO
            // ROS_INFO_STREAM("FAST LAP CONTROL IS READY TO GO.");
            mpc.setTrack(track_xy.X,track_xy.Y);

            x0 = controlNode.initialize();
            ROS_INFO("Starting State:\nx:%lf\ny:%lf\nyaw:%lf\nvx:%lf\nvy:%lf\ns:%lf.", x0.X, x0.Y, x0.phi, x0.vx, x0.vy, x0.s); 
            
            MPCReturn mpc_sol = mpc.runMPC(x0); // Updates s
            controlNode.publishActuation(mpc_sol.u0.dD, mpc_sol.u0.dDelta);
            log.push_back(mpc_sol);
            x0 = integrator.simTimeStep(x0,mpc_sol.u0,jsonConfig["Ts"]); // Updates s and vs
            // x0.s = s_state.s;
            // x0 = controlNode.update(x0, mpc_sol.u0, jsonConfig["Ts"]); // Update state with SLAM data 
            
            firstRun = false;
        }

        else if (controlNode.getFastLapReady())
        {
            ROS_INFO("States:\nx:%lf\ny:%lf\nyaw:%lf\ns:%lf.", x0.X, x0.Y, x0.phi, x0.s);  
            tic();
            MPCReturn mpc_sol = mpc.runMPC(x0); // Updates s
            toc();
            controlNode.publishActuation(mpc_sol.u0.dD, mpc_sol.u0.dDelta);
            log.push_back(mpc_sol);

            x0 = integrator.simTimeStep(x0,mpc_sol.u0,jsonConfig["Ts"]); // Updates s and vs
            // x0.s = s_state.s;
            // x0 = controlNode.update(x0, mpc_sol.u0, jsonConfig["Ts"]); // Update state with SLAM data 

            // ROS_INFO_STREAM("Track Length" << track.path.getLength());

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

        // rate.sleep(); // Wait until Ts
    }

    // MPC mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths);
    // mpc.setTrack(track_xy.X,track_xy.Y);
    // const double phi_0 = std::atan2(track_xy.Y(1) - track_xy.Y(0),track_xy.X(1) - track_xy.X(0));
    // State x0 = {track_xy.X(0),track_xy.Y(0),phi_0,jsonConfig["v0"],0,0,0,0.5,0,jsonConfig["v0"]};
    // for(int i=0;i<jsonConfig["n_sim"];i++)
    // {
    //     MPCReturn mpc_sol = mpc.runMPC(x0);
    //     x0 = integrator.simTimeStep(x0,mpc_sol.u0,jsonConfig["Ts"]);
    //     log.push_back(mpc_sol);
    // }
    // plotter.plotRun(log,track_xy);
}


