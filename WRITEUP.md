# Udacity Path Planning Project

## Author: Adwait Verulkar. Date: 08/28/20. 

### 1. Project Rubric
The goal of the project is to build a path planning state machine for successfully driving an autonomous vehicle on a 3-lane highway. The project requirements have been summarized below.

* drive at least 4.32 miles without incident
* follow the speed limit at all times
* max acceleration and jerk not to exceed 10 m/s^2 and 10 m/s^3
* avoid collisions with other cars
* ability to maintain a legal driving lane
* ability to safely change lanes when required

### 2. Behavior Planner
In order to satisfy the above project, a behavior planner is required. In this project, the behavior planner has been implemented using a state machine. This state machine has 3 states and uses a cost function for state transition.

The 3 states of the behavior planner are as follows

1. Keep Lane (KL)
2. Lane Change Left (LCL)
3. Lane Change Right (LCR)

These states have been implemented using an ```enum``` in the ```Vehicle``` class.
```cpp
class Vehicle {
public:
    enum states {
        KL,
        LCL,
        LCR,
    };
```
The best state is decided by the function ```Vehicle::choose_best_trajectory()```, which is the only function called from the ```main.cpp``` file.
```cpp
          msgJson["next_x"] = ego.choose_best_trajectory()[0];
          msgJson["next_y"] = ego.choose_best_trajectory()[1];
```
This function returns the x and y waypoints required by the simulator in order to move the car around the highway track. The source code for ```Vehicle::choose_best_trajectory``` is as follows.
```cpp
    vector<Vehicle::states> next_states = successor_states(); // Get future states
    double cost, min_cost;
    Vehicle::states final_state;
    min_cost = 9999.0;
    for(int i = 0; i < next_states.size(); i++) {
        cost = calculate_cost(next_states[i]);
        if (min_cost > cost) {
            min_cost = cost;
            final_state = next_states[i]; // State we want to be in
        }
    }
    check_proximity(); // Check if we are too close to vehicle ahead
    
	// Change intended lane for trajectory generation
	if(too_close) {
        if(final_state == Vehicle::LCL) {
            int_lane = curr_lane - 1;
        } else if(final_state == Vehicle::LCR) {
            int_lane = curr_lane + 1;
        }
    }
    return generate_trajectory();
```

The ```Vehicle::KL``` state is the default state of the behavior planner. There are no transitions possible from this state unless there is traffic blocking the ego vehicle's path. This condition is checked constantly by the method ```Vehicle::check_proximity()```. This method checks whether there is traffic in the current lane within 30 meters of current position. If this condition is true, a boolean ```Vehicle::too_close``` is updated to ```true```.
```cpp
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
```
This boolean is also used to control the speed of the car, thereby limiting longitudinal jerk and acceleration.
```cpp
        if(too_close) {
            ref_vel -= 0.05;
        }
        else if(ref_vel < target_vel) {
            ref_vel += 0.05;
        }
```
This also takes care of the speed limit requirements (```target_vel = 49.5``` mph)  and the behavior of the ego vehicle when it is starting from the rest.

The ```predictions``` vector is derived from the ```sensor_fusion``` vector . It represents the future location of all traffic vehicles, considering none of the vehicles change lane. 

```cpp
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
```
The lane change behavior of traffic vehicles can be predicted using a Gaussian Naive Bayes classifier, however, it has not been implemented due to the following reasons.

* lack of proper training data for given highway
* complexity involved in deriving the time derivatives of Frenet coordinates, as this data is not present in the ```sensor_fusion``` vector.

The last two of the important functions in the behavior planning module are ```Vehicle::sucessor_states()``` and ```Vehicle::calculate_cost(Vehicle::states next_state)```.

The last two requirements in the project rubric are pertaining lane changes and lane holding. In order to satisfy these requirements, there needs to be a function that returns the legal states possible given the current localization and map data. ```Vehicle::successor_states()``` method does just that and returns a vector of possible future states that the vehicle can transition to.
```cpp
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
```
The logic makes sure that the ego vehicle does not make left lane change when it is located in the left most lane. Similarly, it also ensures that the vehicle does not drive off the road.

The method ```Vehicle::calculate_cost(Vehicle::states next_state)``` is the transition function of the entire state machine. Provided that a lane change is allowed, whether to change left or right, is decided by lower of the two transition costs. This cost is calculated based on the closest vehicle in the ```future_lane```. ```Vehicle::calculate_cost(Vehicle::states next_state)``` takes the intended future state as an argument and returns the cost associated with this transition. The ```future_lane``` variable is set based on the intended behavior as follows.

```cpp
	if(next_state == Vehicle::KL) {
        future_lane = curr_lane;
    } if(next_state == Vehicle::LCL) {
        future_lane = curr_lane - 1;
    } if(next_state == Vehicle::LCR) {
        future_lane = curr_lane + 1;
    }
```

Once the variable ```future_lane``` is set, the variable ```distance``` and ```speed``` are set to the distance of the closest traffic vehicle and speed of the slowest traffic vehicle in ```future_lane``` respectively. The source code for this is as follows.

```cpp
    double distance = 99999.0, speed = 99999.0;
	double future_s = this->s + this->speed * previous_path_x.size() * 0.02; // where ego is going to be
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
```

Once ```distance``` and ```speed``` are set, the associated cost can be calculated as follows.

```cpp
    if(distance < 25.0) {
        total_cost = 1;
        return total_cost;
    } else {
        distance_cost = 1 - exp(-1 / distance); // Max cost if distance is 0
        speed_cost = 1 - exp(-1 / speed); // Max cost if speed is zero
        total_cost = DISTANCE_WEIGHT * distance_cost + SPEED_WEIGHT * speed_cost;
    }
    return total_cost;
```

As it can be seen in the implementation, if ```distance < 25.0``` meters, then maximum cost of ```1``` is returned. This prevents the ego vehicle from aggressive lane changes or collisions and ensures that there is always a minimum gap of 25 meters between the ego vehicle and traffic before any lane change is attempted.

```DISTANCE_WEIGHT``` and ```SPEED_WEIGHT``` are two constants declared at the start to relatively weight the importance of ```distance_cost``` and ```speed_cost```. For this implementation, there is no importance given to slow speed traffic, and lane changes are strictly based on ```distance```.

```cpp
constexpr double SPEED_WEIGHT = 0.0;
constexpr double DISTANCE_WEIGHT = 1.0 - SPEED_WEIGHT;
```

This means that if the ego vehicle is in the middle lane, it will always shift into a lane which has more room (this data is based on what ```sensor_fusion``` can detect). If ```SPEED_WEIGHT``` is non-zero, the ego vehicle will also take into account speed of traffic in that lane, before making any decisions.

That is the entire logic built into the behavior planner. Once the appropriate behavior is decided, it is sent to the trajectory generation module. This happens at the end of ```Vehicle::choose_best_trajectory()``` method, with the call to ```Vehicle::generate_trajectory()``` method.

###  Trajectory Planner

This module generates the reference points that the car passes through every 0.02 seconds. This code is pretty much the same as the one discussed during the project Q&A session. There is just one minor modification. The code has been split into 2 methods, one for generating the anchor points and other for backfilling high density points by fitting a spline through the anchor points.

Although it doesn't matter for this project, such modular approach benefits when the ego vehicle has to execute many different behaviors such as parking in and out of a spot, city driving, driving in parking lots, and so on. The backfilling code can be common for all these maneuvers, and all the behavior planner has to do is modify anchor points and reference velocities. Modification to reference velocity was discussed in the behavior planner section.

```Vehicle::generate_trajectory()``` is the method that updates the anchor points for trajectory generation. There are 5 anchor points used for fitting a spline in this implementation. 2 of these 5 points are points from the previous trajectory. This ensures that the vehicle follows a smooth trajectory.

```cpp
        ref_x = previous_path_x[previous_path_x.size() -1];
        ref_y = previous_path_y[previous_path_x.size() -1];

        double ref_x_prev = previous_path_x[previous_path_x.size() - 2];
        double ref_y_prev = previous_path_y[previous_path_x.size() - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
```

The remaining 3 points are generated based on the intended lane (chosen by the behavior planner previously). These 3 points spaced equally apart, based on a pre-decided ```horizon_m``` which is 30 meters. Thus the farthest point is roughly 90 m ahead of the vehicles current location. The variable ```horizon_m``` has a significant impact on lateral jerk and acceleration of the vehicle. A shorter distance makes the vehicle switch lanes faster, thereby increasing the jerk and lateral acceleration.

```cpp
	vector<double> next_wp0 = getXY(s + horizon_m, 2 + 4 * int_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(s + 2 * horizon_m, 2 + 4 * int_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(s + 3 * horizon_m, 2 + 4 * int_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
```

Once the anchor points have been calculated, the ```Vehicle::backfill()``` generated the high density points required by the vehicle to follow. This code is also very similar to the one taught in the Q&A video. It first transforms the anchor points to vehicle centric reference frame (origin at ego vehicle) to make the mathematics of point generation simpler.

```cpp
    // Transform to local coordinates
    for(int i = 0; i < anchor_points[0].size(); ++i) {

        // Transform to vehicle coordinates
        double shift_x = anchor_points[0][i] - ref_x;
        double shift_y = anchor_points[1][i] - ref_y;

        anchor_points[0][i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
        anchor_points[1][i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
    }
```

Then it fits a spline from the ```spline.h``` library through the transformed anchor points.

```cpp
	// Initialize a spline
    tk::spline s;
    
    // Fit a spline through the points
    s.set_points(anchor_points[0], anchor_points[1]);
```

Finally it generates whatever points are required to complete the ```trajectory``` vector by using a linear approximation.

```cpp
	double target_x = horizon_m;
    double target_y = s(target_x); // horizon x is distance dead ahead
    double target_dist = distance(target_x, target_y, 0.0, 0.0);

    double x_add_on = 0;

    for(int i = 1; i <= path_size - previous_path_x.size(); ++i) {
		...
        double N = target_dist/(0.02 * ref_vel/2.24);
        double x_point = x_add_on + target_x / N;
        double y_point = s(x_point);
        ...
		// Transform to global coordinates
        ...
        next_path_x.push_back(x_point);
        next_path_y.push_back(y_point);
    }
    trajectory.push_back(next_path_x);
    trajectory.push_back(next_path_y);
```

That concludes the discussion of the logic underlying the entire path planning module.

### Conclusions and Future Work

The path planning algorithm works as expected and tries to optimize the ego vehicles driving behavior for highway driving scenario. A few limitations of this algorithm are summarized below.

* The algorithm assumes traffic vehicles do not change lane, which might lead to occasional collisions. To prevent, the algorithm behavior has been "tuned" to ensure overly safe driving. Optimizations are possible if a Gaussian Naive Bayes Classifier is implemented in the Behavior Planning Module. As a result the vehicle struggles in heavy traffic conditions.
* The vehicle does not consider traffic speed in making decisions about which lane to switch to.
* The ego vehicle speed fluctuates in high traffic conditions. This may be a limitation of the way the simulator requires input data. The vehicle speed is decided based on the spacing of waypoints. As the waypoints have no "speed" attribute themselves, the vehicle follows the waypoints from previous time steps and thus its speed data is obsolete as per the current time step. This phenomenon has been controlled by limiting the number of path points in one time step to 20.