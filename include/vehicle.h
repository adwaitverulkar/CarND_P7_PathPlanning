#pragma once
#include <vector>
#include "classifier.h"
#include "cost.h"
#include "spline.h"

using std::vector;

class Vehicle {
public:
    enum states {
        KL,
        LCL,
        LCR,
        PLCL,
        PLCR
    };
    Vehicle();
    ~Vehicle();

    // store unused trajectory for smoothness
    void set_unused_trajectory(vector<double> previous_x, vector<double> previous_y);
    
    // Returns best cost trajectory by choosing from possible next states
    vector<vector<double>> choose_best_trajectory(vector<vector<double>> sensor_fusion);
    
    // Given a possible next state, generate the appropriate trajectory to realize the next state.
    vector<vector<double>> generate_trajectory(states state, vector<vector<double>> predictions);

    // Generate high density points from anchor points
    vector<vector<double>> backfill(vector<vector<double>> anchor_points, double ref_x, double ref_y, double ref_yaw);
    
    // return possible states from current state
    vector<states> successor_states();

    // return future position in Frenet of all cars on road
    vector<vector<double>> generate_predictions(int horizon);

    double x, y, s, d, yaw, speed, ref_vel;
    int int_lane, curr_lane;
    GNB gnb;
    vector<double> previous_path_x, previous_path_y;
    states curr_state;
    double horizon_x;

    // Vectors for storing map in the Vehicle
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;
};