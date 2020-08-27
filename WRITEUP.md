# Udacity Path Planning Project

### 1. Project Rubric
The goal of the project is to build a path planning state machine for successfully driving an autonomous vehicle on a 3-lane highway. The project requirements have been summarized below.

* drive atleast 4.32 miles without incident
* follow the speed limit at all times
* max acceleration and jerk not to exceed 10 m/s^2^ and 10 m/s^3^
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
This function returns the x and y waypoints required by the simulator in order to move the car around the highway track.
```cpp
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
This boolean is also used to control the speed of the car as follows.
```cpp
        if(too_close) {
            ref_vel -= 0.05;
        }
        else if(ref_vel < target_vel) {
            ref_vel += 0.05;
        }
```
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

The last two of the important functions in the behavior planning module are ```Vehicle::sucessor_states()``` and ```Vehicle::calculate_cost```.

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

The method ```Vehicle::calculate_cost``` is the transition function of the entire state machine. Provided that a lane change is allowed, whether to change left or right, is decided by lower of the two transition costs.


### Trajectory Planner
This module generates the reference points that the car passes through every 0.02 seconds.


###fdsfha