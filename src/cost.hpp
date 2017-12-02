//
//  cost.hpp
//  Behavioural Planner
//
//  Created by Emil Balcu on 12.11.17.
//  Copyright © 2017 Natalia Balcu. All rights reserved.
//

#ifndef cost_hpp
#define cost_hpp

#include <stdio.h>
#include "vehicle.hpp"


using namespace std;

float calculate_cost(Vehicle vehicle, map<int, vector<Vehicle>> predictions, vector<Vehicle> trajectory);

float goal_distance_cost(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions, map<string, float> data);

float inefficiency_cost(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions, map<string, float> data);

float lane_speed(map<int, vector<Vehicle>> predictions, int lane);

map<string, float> get_helper_data(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions);

float collision_cost(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions, map<string, float> data);

float buffer_cost(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions, map<string, float> data);

float logistic(float x);

float get_nearest_distance(vector<Vehicle> trajectory,map<int, vector<Vehicle>> predictions);

#endif /* cost_hpp */
