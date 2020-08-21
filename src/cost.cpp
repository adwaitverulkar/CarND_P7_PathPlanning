#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>

using std::string;
using std::vector;

/**
 * TODO: change weights for cost functions.
 */
const float REACH_GOAL = pow(10,7);
const float EFFICIENCY = pow(10,6);

float goal_distance_cost() {
  return 0.0;
}

float inefficiency_cost() {
  return 0.0;
}

float calculate_cost() {
  return 0.0;
}