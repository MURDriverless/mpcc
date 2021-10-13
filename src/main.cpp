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

// #include "Tests/spline_test.h"
// #include "Tests/model_integrator_test.h"
// #include "Tests/constratins_test.h"
// #include "Tests/cost_test.h"

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
#include <iomanip>

// Statistics Print constants
#define STATROW 16
#define BARWIDTH 30

// Lap constants
#define NUMLAP 10 // Total number of laps before stopping
#define SLOWLAP 1 // Total number of slow laps

// Time testing
std::stack<clock_t> tictoc_stack;

void tic() {
    tictoc_stack.push(clock());
}

double toc() {
    double t = ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
    // std::cout << "Time elapsed: "
    //           << t
    //           << "s, Hz: "
    //           << 1.0/t
    //           << std::endl;
    tictoc_stack.pop();
    return t;
}

// Print Statistics
void printStatistics(mpcc::State x0, Eigen::Vector4d command, mpcc::ArcLengthSpline track_, 
    Eigen::Vector4d lapstats, double lapTime, double t, Eigen::Vector2d error_count, Eigen::Vector2d wheel_constraint)
{
    // Decimals
    std::cout << std::fixed; 
    std::cout << std::setprecision(5);

    std::cout << "States:" << std::endl;
    std::cout << "x: " << x0.X << std::endl;
    std::cout << "y: " << x0.Y << std::endl;
    std::cout << "theta: " << x0.phi << std::endl;
    std::cout << "s: " << x0.s << ", max length: " << track_.getLength() << std::endl;
    std::cout << "vx: " << x0.vx << std::endl;
    std::cout << "vy: " << x0.vy << std::endl;
    std::cout << "w: " << x0.r << std::endl;
    std::cout << "D: " << command(0) << std::endl;
    std::cout << "dD: " << command(2) << std::endl;
    std::cout << "delta: " << command(1) << std::endl;
    std::cout << "dDelta: " << command(3) << std::endl;
    std::cout << "Controller Status: " << std::endl;
    std::cout << "Solve time: " << t << "s, Hz: " << 1.0/t << std::endl;
    std::cout << "exitflag1 count: " << int(error_count(0)) << ", exitflag2 count: " << int(error_count(1)) << std::endl;
    // std::cout << "Constraint Violation count: " << int(wheel_constraint(0)) << ", Max Violation: " << wheel_constraint(1) << std::endl;
    std::cout << "Previous Lap time: " << lapTime << ", average: " << lapstats[1] << ", min: " << lapstats[2] << ", max: " << lapstats[3] << std::endl;

    std::cout << "lap " << int(lapstats(0)) << " progress: [";
    int pos = BARWIDTH * x0.s/track_.getLength();
    for (int i = 0; i < BARWIDTH; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(x0.s/track_.getLength() * 100.0) << "%   ";
    
    // Set stationary
    std::cout << "\r";
    for (int i = 0; i < STATROW; i++)
        std::cout << "\033[F";
    std::cout.flush();
}

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "alex_mpcc");

    // Get parameters from CLI
    bool comm = atoi(argv[1]);
    bool skip = atoi(argv[2]);

    using namespace mpcc;
    std::ifstream iConfig("/workspace/src/simulation/alexmpcc/src/Params/config.json");
    json jsonConfig;
    iConfig >> jsonConfig;

    ROS_INFO_STREAM("FAST LAP CONTROL - MPCC - Initialized configuration.");

    PathToJson json_paths {jsonConfig["model_path"],
                           jsonConfig["cost_path"],
                           jsonConfig["bounds_path"],
                           jsonConfig["track_path"],
                           jsonConfig["normalization_path"]};

    // std::cout << testSpline() << std::endl;
    // std::cout << testArcLengthSpline(json_paths) << std::endl;

    // std::cout << testIntegrator(json_paths) << std::endl;
    // std::cout << testLinModel(json_paths) << std::endl;

    // std::cout << testAlphaConstraint(sjson_paths) << std::endl;
    // std::cout << testTireForceConstraint(json_paths) << std::endl;
    // std::cout << testTrackConstraint(json_paths) << std::endl;

    // std::cout << testCost(json_paths) << std::endl;

    Integrator integrator = Integrator(jsonConfig["Ts"],json_paths);
    Plotting plotter = Plotting(jsonConfig["Ts"],json_paths);

    // Track track = Track(json_paths.track_path);
    // TrackPos track_xy = track.getTrack();

    // Create ROS Node stuff
    FastLapControlNode controlNode = FastLapControlNode(json_paths);
    bool firstRun = true;

    // ROS INFO
    ROS_INFO_STREAM("FAST LAP CONTROL - MPCC - Initializing MPC.");

    // Empty Objects
    MPC mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths);
    State x0;
    std::list<MPCReturn> log;
    Track track;
    TrackPos track_xy;
    ArcLengthSpline track_; // parameterized track

    // Set Ts
    ros::Rate rate(jsonConfig["Hz"]); // 1/Ts, Ts = 0.1

    // Lap count and time
    double cur_s = 0;
    Eigen::Vector4d fastlap_stats; // 0 for lap count, 1 for avg lap time, 2 for min lap time, 3 for max lap time
    double lapTime[NUMLAP]; // all lap time
    auto startTime = std::chrono::high_resolution_clock::now(); // Start counting for slow lap

    // Debug and stats
    int simcount = 0; // Set max number of simulation
    int count = 0; // For no comms lap time stats
    Eigen::Vector2d wheel_violate; // Wheel Constraint Testing, 0 for count, 1 for max
    double t = 0; // solve time and Hz
    double max_t = 0; // solve time and Hz
    Eigen::Vector2d error_count; // count exit error 1 and 2

    ROS_INFO_STREAM("FAST LAP CONTROL STARTED.");

    while(ros::ok())
    {
        // Lap Count
        if (fastlap_stats(0) == NUMLAP) // Finished all 10 laps
        {
            std::vector<double> plot_lapTime(std::begin(lapTime), std::end(lapTime));
            if (x0.s > 1) // Cross a little bit then stop
            {
                controlNode.publishActuation(0, 0); // Command desired velocity
                if (comm)
                {
                    plotter.plotRun(log, track_xy, track_, plot_lapTime);
                    return 0; // Finish race
                }
                else
                {
                    controlNode.fastlapready = false; // end fast lap
                    double mean_time = 0.0;
                    double max_time = 0.0;
                    for(MPCReturn log_i : log)
                    {
                        mean_time += log_i.time_total;
                        if(log_i.time_total > max_time)
                            max_time = log_i.time_total;
                    }
                    // std::cout << "mean nmpc time " << mean_time/double(jsonConfig["n_sim"]) << std::endl;
                    for (int i = 0; i < STATROW+1; i++)
                        std::cout << std::endl;
                    std::cout << "mean nmpc time " << mean_time/simcount << std::endl;
                    std::cout << "max nmpc time " << max_time << std::endl;
                    std::cout << "wheel violation count " << wheel_violate(0) << std::endl;
                    std::cout << "max wheel violation " << wheel_violate(1) << std::endl;
                    std::cout << "exitflag 1 count " << error_count(0) << std::endl;
                    std::cout << "exitflag 2 count " << error_count(1) << std::endl;
                    plotter.plotRun(log, track_xy, track_, plot_lapTime);
                    plotter.plotSim(log, track_xy, track_);
                    return 0;
                }
            }
        }

        if (!comm) // Simple sim test without Gazebo/RVIZ
        {
            if (simcount > jsonConfig["n_sim"]) // reach max sim count
            {
                controlNode.publishActuation(0, 0);
                controlNode.fastlapready = false; // end fast lap
                double mean_time = 0.0;
                double max_time = 0.0;
                for(MPCReturn log_i : log)
                {
                    mean_time += log_i.time_total;
                    if(log_i.time_total > max_time)
                        max_time = log_i.time_total;
                }
                // std::cout << "mean nmpc time " << mean_time/double(jsonConfig["n_sim"]) << std::endl;
                std::cout << "mean nmpc time " << mean_time/simcount << std::endl;
                std::cout << "max nmpc time " << max_time << std::endl;
                std::vector<double> plot_lapTime(std::begin(lapTime), std::end(lapTime));
                plotter.plotRun(log, track_xy, track_, plot_lapTime);
                plotter.plotSim(log, track_xy, track_);
                return 0;
            }
            else
            {
                simcount++;
                count++;
                // std::cout << "count: " << count << std::endl;
            }
        } 
        if (skip)
        {
            controlNode.fastlapready = true; // If want start fast lap immediately
        }
        // Update 
        ros::spinOnce();

        // Fast Lap Control is ready
        if (controlNode.getFastLapReady())
        {
            if (firstRun)
            {
                // ROS INFO
                ROS_INFO_STREAM("FAST LAP CONTROL IS READY TO GO.");
                
                if (skip)
                {
                    track = Track(json_paths.track_path);
                    track_xy = track.getTrack();
                    const double phi_0 = std::atan2(track_xy.Y(1) - track_xy.Y(0),track_xy.X(1) - track_xy.X(0));
                    x0.set(track_xy.X(0),track_xy.Y(0),phi_0,jsonConfig["v0"],0,0,0,0,0,jsonConfig["v0"]);
                }
                else
                {
                    track = controlNode.generateTrack();
                    track_xy = track.getTrack();
                    x0 = controlNode.initialize();
                }
                
                track_ = mpc.setTrack(track_xy.X,track_xy.Y);

                ROS_INFO("Starting State:\nx:%lf\ny:%lf\nyaw:%lf\nvx:%lf\nvy:%lf\ns:%lf.", x0.X, x0.Y, x0.phi, x0.vx, x0.vy, x0.s); 

                firstRun = false;
                simcount = 0;
                count = 0;
            }
            
            tic();
            MPCReturn mpc_sol = mpc.runMPC(x0, error_count); // Updates s
            t = toc();
            log.push_back(mpc_sol);

            x0 = integrator.simTimeStep(x0, mpc_sol.u0,jsonConfig["Ts"]); // Update state with RK4
            controlNode.publishActuation(x0.D, x0.delta);

            Eigen::Vector4d command(x0.D, x0.delta, mpc_sol.u0.dD, mpc_sol.u0.dDelta);

            if(comm)
            {
                x0 = controlNode.update(x0, mpc_sol.u0, jsonConfig["Ts"]); // Update state with SLAM data 
            }

            // Print stats
            if (simcount > (double)jsonConfig["Hz"]) // after 1s for clean print
            {
                printStatistics(x0, command, track_, fastlap_stats, lapTime[int(fastlap_stats(0))-1], t, error_count, wheel_violate);
            }
            controlNode.publishRVIZ(mpc_sol.mpc_horizon, track_); // Publish RVIZ

            // Lap stats
            if((50*x0.s) < cur_s && x0.s < 10) // Finished a lap, progress resets
            {
                auto tnow = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(tnow - startTime);
                if(comm) // running Gazebo, rate.sleep() in effect, can use real time
                {
                    lapTime[(int)fastlap_stats(0)] = time_span.count();
                }
                else // have to manually determine lap time taking into consideration controller sampling time
                {
                    lapTime[(int)fastlap_stats(0)] = count * (double)jsonConfig["Ts"];
                    count = 0;
                }
                fastlap_stats(0)++;
                // Ignore first lap for avg count as its mixed with slow lap
                if (fastlap_stats(0) > SLOWLAP)
                {
                    double total_lapTime = 0;
                    for (int i = 1; i < fastlap_stats(0); i ++)
                    {
                        fastlap_stats(2) = lapTime[i] < fastlap_stats(2) ? lapTime[i] : fastlap_stats(2);
                        fastlap_stats(3) = lapTime[i] > fastlap_stats(3) ? lapTime[i] : fastlap_stats(3);
                        total_lapTime += lapTime[i];
                    }
                    if (fastlap_stats(2) < 1) // # Initialize
                    {
                        fastlap_stats(2) = fastlap_stats(3);
                    }
                    fastlap_stats(1) = total_lapTime/int(fastlap_stats(0)-1);
                }
                cur_s = 0;
                startTime = std::chrono::high_resolution_clock::now();
            }
            else
            {
                cur_s = x0.s; // Save new progress
            }
        }

        if(comm)
        {
            rate.sleep(); // Wait until Ts
        }
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


