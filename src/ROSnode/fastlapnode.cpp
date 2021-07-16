/**
 * ROS Node for Fast Lap Control
 * 
 * SUBSCRIPTIONS
 * SLAM provides vehicle state
 * Planner provides map on transition, which has inner/outer cone position and centerline reference
 * SlowLapFollower provides final actuation output, so that transition is smooth
 * 
 * PUBLISHERS
 * transitionPublisher, to signify fast lap control is ready for transition and slow lap control can stop
 * actuationPublisher, to control the vehicle
 * 
 * Current Version by Kheng Yu Yeoh, contact @ khengyu_061192@hotmail.com
 * 
 */

#include "fastlapnode.h"
#include <External/Json/include/nlohmann/json.hpp> // To write JSON track
using json = nlohmann::json;

// To write csv file of map
#include <iostream>
#include <fstream>

// Constructor
FastLapControlNode::FastLapControlNode()
{
    this->mapready = false;
    this->fastlapready = false;

    this->slamSubscriber = nh.subscribe(ODOM_TOPIC, 1, &FastLapControlNode::slamCallback, this);
    this->mapSubscriber = nh.subscribe(MAP_TOPIC, 1, &FastLapControlNode::mapCallback, this);
    this->finalActuationSubscriber = nh.subscribe(ACTUATION_TOPIC, 1, &FastLapControlNode::finalActuationCallback, this);
    this->actuationPublisher = nh.advertise<mur_common::actuation_msg>(ACTUATION_TOPIC, 1);
    this->transitionPublisher = nh.advertise<mur_common::transition_msg>(TRANSITION_TOPIC, 1);
}

// Obtain vehicle states from SLAM
// topic: /mur/slam/Odom
// msg: nav_msgs/Odometry
void FastLapControlNode::slamCallback(const nav_msgs::Odometry& msg)
{
    // this->yaw = atan2(msg.pose.pose.position.y - this->y, msg.pose.pose.position.x - this->x);
    this->x = msg.pose.pose.position.x;
    this->y = msg.pose.pose.position.y;

    double q_x = msg.pose.pose.orientation.x;
    double q_y = msg.pose.pose.orientation.y;
    double q_z = msg.pose.pose.orientation.z;
    double q_w = msg.pose.pose.orientation.w;
    
    this->yaw = 2.0 * std::asin(std::abs(q_z)) * sign(q_z) * sign(q_w);
    // this->yaw = Quart2EulerYaw(q_x, q_y, q_z, q_w);

    this->vx = msg.twist.twist.linear.x;
    this->vy = msg.twist.twist.linear.y;
    this->wz = msg.twist.twist.angular.z;
}

// Obtain track boundary from planner
// topic: /mur/planner/map
// msg: mur_common/map_msg
void FastLapControlNode::mapCallback(const mur_common::map_msg& msg)
{
    if (msg.mapready && !this->mapready) // Only update once when mapping is done
    {
        std::vector<double> x_o(msg.x_o.begin(), msg.x_o.end());
        std::vector<double> y_o(msg.y_o.begin(), msg.y_o.end());
        std::vector<double> x_i(msg.x_i.begin(), msg.x_i.end());
        std::vector<double> y_i(msg.y_i.begin(), msg.y_i.end());
        std::vector<double> x(msg.x.begin(), msg.x.end());
        std::vector<double> y(msg.y.begin(), msg.y.end());
        this->x_outer = x_o;
        this->y_outer = y_o;
        this->x_inner = x_i;
        this->y_inner = y_i;
        this->x_centre = x;
        this->y_centre = y;

        this->mapready = msg.mapready;

        if (this->mapready)
        {
            ROS_INFO_STREAM("Obtained MAP!, now MAPREADY.");
        }
    }
}

// // Obtain centerline from planner
// // topic: /mur/planner/path
// // msg: mur_common/path_msg 
// void FastLapControlNode::pathCallback(const mur_common::path_msg::ConstPtr& msg)
// {
//     if (this->mapready && !this->pathready) // Only update once when mapping is done and path not ready
//     {
//         this->x_centre = msg.x;
//         this->y_centre = msg.y;
//         this->pathready = true;
//     }
// }

// Obtain final actuation output from slow lap, so it can transition smoothly
// Also publishes fastlapready, so slow lap follower can stop publishing actuation_msg
// topic: /mur/control/actuation
// msg: mur_common/actuation_msg
void FastLapControlNode::finalActuationCallback(const mur_common::actuation_msg& msg)
{
    // Only update once when mapping is done
    if (this->mapready && !this->fastlapready) 
    {
        ROS_INFO_STREAM("MAP READY and Receiving final actuation.");
        this->accel_D = msg.acceleration_threshold;
        this->steering_angle = msg.steering;
        this->fastlapready = true;

        if (this->fastlapready)
        {
            ROS_INFO_STREAM("Obtained FINAL ACTUATION!, now FASTLAPREADY.");
            ROS_INFO_STREAM("Final actuation is " << accel_D);
            ROS_INFO_STREAM("Final steering is " << steering_angle);
        }
    }

    // Initialize msg and publish
    mur_common::transition_msg transition_msg;
    transition_msg.fastlapready = this->fastlapready;
    transitionPublisher.publish(transition_msg);
}

// Publisher function to publish actuation output
void FastLapControlNode::publishActuation(double accel_D, double steering_angle)
{
    // Initialize msg
    mur_common::actuation_msg act_msg;

    // Set msg
    act_msg.acceleration_threshold = accel_D;
    act_msg.steering = steering_angle;
    
    // ROS INFO
    ROS_INFO("PUBLISHING COMMANDS, ACCEL: %lf, STEER: %lf\n", accel_D, steering_angle);
    
    // Publish msg
    actuationPublisher.publish(act_msg);
}

// Get Functions
bool FastLapControlNode::getFastLapReady(){
    return this->fastlapready;
}

// Track generation function, using obtained map inputs
// returns: the mapped track of type Track
mpcc::Track FastLapControlNode::generateTrack() 
{
    mpcc::Track track = mpcc::Track(this->x_outer, this->y_outer, 
                                    this->x_inner, this->y_inner, 
                                    this->x_centre, this->y_centre);

    // int o_count = 0;
    // int i_count = 0;
    // int c_count = 0;
    // ROS_INFO_STREAM("Track generated with these parameters");
    // ROS_INFO_STREAM("X_Outer: ");
    // for (auto i = this->x_outer.begin(); i != this->x_outer.end(); ++i)
    // {
    //     std::cout << *i << ' ';
    //     o_count++;
    // }
    // ROS_INFO_STREAM("Y_Outer: ");
    // for (auto i = this->y_outer.begin(); i != this->y_outer.end(); ++i)
    // {
    //     std::cout << *i << ' ';
    // }
    // ROS_INFO_STREAM("X_Inner: ");
    // for (auto i = this->x_inner.begin(); i != this->x_inner.end(); ++i)
    // {
    //     std::cout << *i << ' ';
    //     i_count++;
    // }
    // ROS_INFO_STREAM("Y_Inner: ");
    // for (auto i = this->y_inner.begin(); i != this->y_inner.end(); ++i)
    // {
    //     std::cout << *i << ' ';
    // }
    // ROS_INFO_STREAM("X_Centre: ");
    // for (auto i = this->x_centre.begin(); i != this->x_centre.end(); ++i)
    // {
    //     std::cout << *i << ' ';
    //     c_count++;
    // }
    // ROS_INFO_STREAM("Y_Centre: ");
    // for (auto i = this->y_centre.begin(); i != this->y_centre.end(); ++i)
    // {
    //     std::cout << *i << ' ';
    //     c_count++;
    // }
    // ROS_INFO_STREAM("\nOuter num: " << o_count << ". Inner num: " << i_count << ". Centre num: " << c_count); //25 30 118

    // // Write current track into csv
    // std::ofstream trackfile;
    // trackfile.open("/workspace/track.csv");
    // trackfile << "x_o,y_o,x_i,y_i,x,y,left as outer and right as inner for now.\n";
    // for (int i = 0; i < c_count; i++)
    // {
    //     if (i > o_count)
    //     {
    //         trackfile << "0,0,";
    //     }
    //     else
    //     {
    //         trackfile << this->x_outer[i] << "," << this->y_outer[i] << ",";
    //     }

    //     if (i > i_count)
    //     {
    //         trackfile << "0,0,";
    //     }
    //     else
    //     {
    //         trackfile << this->x_inner[i] << "," << this->y_inner[i] << ",";
    //     }

    //     trackfile << this->x_centre[i] << "," << this->y_centre[i] << ",\n";
    // }
    
    // trackfile.close();

    // // Write current track as json
    // json json_track;
    // json_track["X_o"] = x_outer;
    // json_track["X_i"] = x_inner;
    // json_track["Y_o"] = y_outer;
    // json_track["Y_i"] = y_inner;
    // json_track["X"] = x_centre;
    // json_track["Y"] = y_centre;

    // std::ofstream file("/workspace/track.json");
    // file << json_track;

    return track;
}


// State initialization function
// returns: initial state of vehicle after transition from slow lap
mpcc::State FastLapControlNode::initialize()
{
    mpcc::State x = {
        // this->x,
        // this->y,
        // this->yaw,
        -13.00, // eufs starting at orange cone (x)
        10.30, // eufs starting at orange cone (y)
        0.00, // eufs starting at orange cone (yaw)
        // -15.30, // eufs starting when map connected (x)
        // 5.60, // eufs starting when map connected (y)
        // 1.57, // eufs starting when map connected (yaw) 
        // 0.0, // dennis track at top (x)
        // 0.0, // dennis track at top (y)
        // 0.0, // dennis track at top (yaw)
        // 25.0, // dennis track difficult (x)
        // -58.0, // dennis track difficult (y)
        // -0.7, // dennis strack difficult (yaw)
        // -10.30, // alex RC (x)
        // 13.60, // alex RC (y)
        // -0.70, // alex RC (yaw)
        // -31.30, // alex full (x)
        // -14.60, // alex full (y)
        // -0.70, // alex full (yaw)
        this->vx,
        // 1.0, // slow lap end speed
        this->vy,
        this->wz,  
        0.0,  // s
        this->accel_D,
        this->steering_angle,
        this->vx // vs
        // 1.0, // slow lap end speed
    };

    return x;
}

// Update function
// input: current vehicle state
// returns: state with updated SLAM outputs
mpcc::State FastLapControlNode::update(const mpcc::State& x0, const mpcc::Input& u0, double Ts)
{
    mpcc::State x_new;
    x_new.X = this->x;
    x_new.Y = this->y;
    x_new.phi = this->yaw;
    x_new.vx = this->vx;
    x_new.vy = this->vy;
    x_new.r = this->wz;
    // Copy over from RK4
    x_new.s = x0.s;
    x_new.D = x0.D;
    x_new.delta = x0.delta;
    x_new.vs = x0.vs;
    
    // x_new.s = x0.s + Ts*x0.vs + Ts*Ts/2*u0.dVs; // RK4 update
    // x_new.D = x0.D + Ts * u0.dD;
    // x_new.delta = x0.delta + Ts * u0.dDelta; 
    // x_new.vs = x0.vs + Ts * u0.dVs;

    return x_new;
}

// Returns 1 if positive, 0 if 0, -1 if negative
// For use in Quarternion conversion to Radians
int sign(double x)
{
    if (x>0)
    {
        return 1;
    }
    else if (x==0)
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

// Converts from Quartenion to Euler Yaw
// From https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
double Quart2EulerYaw(double q_x, double q_y, double q_z, double q_w)
{
    double siny_cosp = 2.0 * (q_w * q_z + q_x * q_y);
    double cosy_cosp = 1.0 - (2.0 * (q_y * q_y + q_z * q_z));
    return std::atan2(siny_cosp, cosy_cosp);
}