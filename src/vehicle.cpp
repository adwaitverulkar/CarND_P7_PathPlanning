#include "vehicle.h"
#include "helpers.h"
#include <fstream>
#include <iostream>

using std::cout;
using std::endl;

constexpr double SPEED_WEIGHT = 0.0;
constexpr double DISTANCE_WEIGHT = 1.0 - SPEED_WEIGHT;

Vehicle::Vehicle() {
    this->curr_state = Vehicle::KL;
    this->ref_vel = 0.0; // mph
    this->target_vel = 49.5; // mph
    this->int_lane = 1;
    this->curr_lane = 1;
    this->curr_state = Vehicle::KL;
    int prev_size = previous_path_x.size();
    this->path_size = 20; // points in the future
    horizon_m = 30; // meters
    horizon_t = 1; // seconds
    too_close = false;
    
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

vector<vector<double>> Vehicle::choose_best_trajectory() {
    
    vector<Vehicle::states> next_states = successor_states();
    double cost, min_cost;
    Vehicle::states final_state;
    min_cost = 9999.0;
    for(int i = 0; i < next_states.size(); i++) {
        cost = calculate_cost(next_states[i]);
        if (min_cost > cost) {
            min_cost = cost;
            final_state = next_states[i];
        }
    }
    check_proximity(); // Check if we are too close to vehicle ahead
    if(too_close) {
        if(final_state == Vehicle::LCL) {
            int_lane = curr_lane - 1;
        } else if(final_state == Vehicle::LCR) {
            int_lane = curr_lane + 1;
        }
    }
    return generate_trajectory();
}
vector<vector<double>> Vehicle::generate_trajectory() {
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
    vector<double> next_wp0 = getXY(s + horizon_m, 2 + 4 * int_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(s + 2 * horizon_m, 2 + 4 * int_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(s + 3 * horizon_m, 2 + 4 * int_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    vector<vector<double>> anchor_points, trajectory;
    
    anchor_points.push_back(ptsx);
    anchor_points.push_back(ptsy);

    trajectory = backfill(anchor_points, ref_x, ref_y, ref_yaw);

    return trajectory;
}
vector<vector<double>> Vehicle::backfill(vector<vector<double>> anchor_points, double ref_x, double ref_y, double ref_yaw) {
    
    // Transform to local coordinates
    for(int i = 0; i < anchor_points[0].size(); ++i) {

        // Transform to vehicle coordinates
        double shift_x = anchor_points[0][i] - ref_x;
        double shift_y = anchor_points[1][i] - ref_y;

        anchor_points[0][i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
        anchor_points[1][i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
    }
    
    // Initialize a spline
    tk::spline s;
    
    // Fit a spline through the points
    s.set_points(anchor_points[0], anchor_points[1]);

    vector<vector<double>> trajectory;
    vector<double> next_path_x;
    vector<double> next_path_y;

    for(int i = 0; i < previous_path_x.size(); ++i) {
        next_path_x.push_back(previous_path_x[i]);
        next_path_y.push_back(previous_path_y[i]);
    }
    double target_x = horizon_m;
    double target_y = s(target_x); // horizon x is distance dead ahead
    double target_dist = distance(target_x, target_y, 0.0, 0.0);

    double x_add_on = 0;

    for(int i = 1; i <= path_size - previous_path_x.size(); ++i) {

        // Update ref_vel for every point to get efficient speed control
        if(too_close) {
            ref_vel -= 0.05;
        }
        else if(ref_vel < target_vel) {
            ref_vel += 0.05;
        }
        double N = target_dist/(0.02 * ref_vel/2.24);
        double x_point = x_add_on + target_x / N;
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
void Vehicle::generate_predictions(vector<vector<double>> sensor_fusion) {
    // clear previous predictions
    predictions.clear();

    //0 - ID, 1 - x, 2 - y, 3 - x_dot, 4 - y_dot, 5 - s, 6 - d
    for(int i = 0; i < sensor_fusion.size(); i++) {
        vector<double> sample, prediction;

        double traffic_xdot = sensor_fusion[i][3];
        double traffic_ydot = sensor_fusion[i][4];

        double traffic_s = sensor_fusion[i][5];
        double traffic_d = sensor_fusion[i][6];

        double traffic_speed = distance(traffic_xdot, traffic_ydot, 0.0, 0.0);

        prediction.push_back(traffic_s + traffic_speed * 0.02 * previous_path_x.size());

        // Assume traffic never changes lane
        prediction.push_back(traffic_d);

        // Push velocity to follow vehicle ahead
        prediction.push_back(traffic_speed);
        
        predictions.push_back(prediction);
    }
}
          
vector<Vehicle::states> Vehicle::successor_states() {
    vector<Vehicle::states> next_states;
    next_states.push_back(Vehicle::KL);
    if(this->curr_state == Vehicle::KL) {
        if(curr_lane > 0)
            next_states.push_back(Vehicle::LCL);
        if(curr_lane < 2)
            next_states.push_back(Vehicle::LCR);
    } 
    return next_states;
}

void Vehicle::check_proximity() {
    for(int i = 0; i < predictions.size(); ++i) {
        double traffic_s = predictions[i][0];
        double traffic_d = predictions[i][1];
        if((traffic_d > (2 + 4*curr_lane-2)) && (traffic_d < (2 + 4*curr_lane+2))) {
            if((traffic_s > s) && (traffic_s - s) < 30.0) {
                too_close = true;
            }
        }
    }
}
double Vehicle::calculate_cost(Vehicle::states next_state) {
    double distance = 99999.0, speed = 99999.0, distance_cost, speed_cost, total_cost = 1.0;
    int future_lane;
    bool assigned = false;
    double future_s = this->s + this->speed * previous_path_x.size() * 0.02;

    if(next_state == Vehicle::KL) {
        future_lane = curr_lane;
    } if(next_state == Vehicle::LCL) {
        future_lane = curr_lane - 1;
    } if(next_state == Vehicle::LCR) {
        future_lane = curr_lane + 1;
    }

    for(int i = 0; i < predictions.size(); ++i) {
        double traffic_s = predictions[i][0];
        double traffic_d = predictions[i][1];
        double traffic_speed = predictions[i][2];
        if((traffic_d > (2 + 4*future_lane-2)) && (traffic_d < (2 + 4*future_lane+2))) {
            if(abs(traffic_s - future_s) < distance) {
                distance = abs(traffic_s - future_s);
            }
            if(traffic_speed < speed) {
                speed = traffic_speed;
            }
        }
    }
    if(distance < 25.0) {
        total_cost = 1;
        return total_cost;
    } else {
        distance_cost = 1 - exp(-1 / distance);
        speed_cost = 1 - exp(-1 / speed);
        total_cost = DISTANCE_WEIGHT * distance_cost + SPEED_WEIGHT * speed_cost;
    }
    return total_cost;
}

        

        

    


