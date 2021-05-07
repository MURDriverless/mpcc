/**
 * Header file for the FastLapControl ROS node,
 * Subscribes to /mur/slam/Odom topic from SLAM
 * Subscribes to /mur/planner/map topic from planner_exploratory
 * Subscribes to /mur/control/actuation from slow lap control
 * Publishes /mur/control/transition to slow lap control
 * Publishes /mur/control/actuation to actuation
 * 
 * Current Version by Kheng Yu Yeoh, contact @ khengyu_061192@hotmail.com
 * 
 */

#ifndef FASTLAPCONTROLNODE_H
#define FASTLAPCONTROLNODE_H

// General & MPCC includes
#include <cmath>
#include "../models/state.h"
#include "../splines/track.h"
// #include <unsupported/Eigen/MatrixFunctions>
// #include "../models/model_interface.h"
// #include "../models/model_params.h"

// ROS includes
#include <ros/ros.h>
#include <nav_msgs/Odometry.h> // Msg for /mur/slam/Odom topic
#include "mur_common/map_msg.h" // Msg for /mur/planner/map topic
#include "mur_common/actuation_msg.h" // Msg for /mur/control/actuation topic
#include "mur_common/transition_msg.h" // Msg for /mur/control/transition topic
// #include <nav_msgs/Path.h> 
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/Point.h>
// #include "mur_common/cone_msg.h"
// #include "mur_common/diagnostic_msg.h"

#define ODOM_TOPIC "/mur/slam/Odom"
#define MAP_TOPIC "/mur/planner/map"
#define ACTUATION_TOPIC "/mur/control/actuation"
#define TRANSITION_TOPIC "/mur/control/transition"
// #define CONE_TOPIC "/mur/slam/cones"
// #define PATH_VIZ_TOPIC "/mur/planner/path_viz"
// #define HEALTH_TOPIC "/mur/planner/topic_health"

class FastLapControlNode {
    private:
    // Determine if obtained map and whether controller is ready to run
    bool mapready;
    // bool pathready;
    bool fastlapready;

    // Vehicle States From SLAM
    double x;
    double y;
    double yaw; // Convert from Quartenion to Euler
    double vx;
    double vy;
    double wz;
    double accel_D;
    double steering_angle;

    // Track parameter from Planner TODO: Inner Outer Cone Position in Planner
    std::vector<double> x_outer;
    std::vector<double> y_outer;
    std::vector<double> x_inner;
    std::vector<double> y_inner;
    std::vector<double> x_centre;
    std::vector<double> y_centre;

    // ROS stuff
    ros::NodeHandle nh; // Create its specific node handler
    ros::Subscriber slamSubscriber;
    ros::Subscriber mapSubscriber; // TODO: New topic publish from planner
    // ros::Subscriber pathSubscriber;
    ros::Subscriber finalActuationSubscriber; // To obtain final actuation output of slow lap before transition
    ros::Publisher actuationPublisher;
    ros::Publisher transitionPublisher; // Publish so that slow lap can stop publishing actuation

    public:    
    // Constructor
    FastLapControlNode();

    // Callback Function
    void slamCallback(const nav_msgs::Odometry& msg);
    void mapCallback(const mur_common::map_msg& msg);
    void finalActuationCallback(const mur_common::actuation_msg& msg);
    // void pathCallback();
    
    // Publish Function
    void publishActuation(double accel_D, double steering_angle);

    // Get Functions
    bool getFastLapReady();

    // Function for Mapped Track generation
    Track generateTrack();

    // Functions for vehicle state
    State initialize();
    State update(const State& x0);
};

// Returns 1 if positive, 0 if 0, -1 if negative
// For use in Quarternion conversion to Radians
int sign(double x);

// Converts from Quartenion to Euler Yaw
// From https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
double Quart2EulerYaw(double q_x, double q_y, double q_z, double q_w);

#endif