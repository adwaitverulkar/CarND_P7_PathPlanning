#include "vehicle.h"
#include "helpers.h"
#include <fstream>
#include <iostream>

using std::cout;
using std::endl;

Vehicle::Vehicle() {
    this->curr_state = Vehicle::KL;
    this->ref_vel = 10.0;
    this->int_lane = 1;
    this->curr_lane = 1;
    this->curr_state = Vehicle::KL;
    int prev_size = previous_path_x.size();
    horizon_x = 30; // meters
    
    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";

    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }
}

Vehicle::~Vehicle() {}

vector<vector<double>> Vehicle::choose_best_trajectory(vector<vector<double>> sensor_fusion) {
    //placeholder code
    vector<vector<double>> predictions = {{1.0, 2.0}, {1.0, 2.0}};
    return generate_trajectory(curr_state, predictions);
}
vector<vector<double>> Vehicle::generate_trajectory(states state, 
                                                    vector<vector<double>> predictions) {
    vector<double> ptsx, ptsy;

    double ref_x = x, ref_y = y;
    double ref_yaw = deg2rad(yaw);
    if(previous_path_x.size() < 2) {
        double prev_car_x = x - cos(yaw);
        double prev_car_y = y - sin(yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(y);
    } 
    else {
        ref_x = previous_path_x[previous_path_x.size() -1];
        ref_y = previous_path_y[previous_path_x.size() -1];

        double ref_x_prev = previous_path_x[previous_path_x.size() - 2];
        double ref_y_prev = previous_path_y[previous_path_x.size() - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }
    // Add 30m spaced out points ahead of starting points
    vector<double> next_wp0 = getXY(s + horizon_x, 2 + 4 * int_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(s + 2 * horizon_x, 2 + 4 * int_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(s + 3 * horizon_x, 2 + 4 * int_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for(int i = 0; i < ptsx.size(); ++i) {

        // Transform to vehicle coordinates
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
        ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);

    }
    // Initialize a spline
    tk::spline s;

    // Fit a spline through the points
    s.set_points(ptsx, ptsy);

    vector<vector<double>> trajectory;
    vector<double> next_path_x;
    vector<double> next_path_y;

    for(int i = 0; i < previous_path_x.size(); ++i) {
        next_path_x.push_back(previous_path_x[i]);
        next_path_y.push_back(previous_path_y[i]);
    }

    double horizon_y = s(horizon_x); // horizon x is distance dead ahead
    double target_dist = distance(horizon_x, horizon_y, 0.0, 0.0);

    double x_add_on = 0;

    for(int i = 1; i <= 50 - previous_path_x.size(); ++i) {

        double N = target_dist/(0.02 * ref_vel/2.24);
        double x_point = x_add_on + horizon_x / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // Transform to global coordinates
        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
        x_point += ref_x;
        y_point += ref_y;
        next_path_x.push_back(x_point);
        next_path_y.push_back(y_point);
    }
    trajectory.push_back(next_path_x);
    trajectory.push_back(next_path_y);
    return trajectory;
}
void Vehicle::set_unused_trajectory(vector<double> previous_x, vector<double> previous_y) {
    this->previous_path_x = previous_x;
    this->previous_path_y = previous_y;
}

          

        
        

        

    


