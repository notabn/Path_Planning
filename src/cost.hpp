//
//  cost.hpp
//  Behavioural Planner
//
//  Adapted cost class from Udacity


#ifndef cost_hpp
#define cost_hpp

#include <stdio.h>
#include "vehicle.hpp"


using namespace std;

float calculate_cost(Vehicle vehicle, map<int, vector<Vehicle>> predictions, vector<Vehicle> trajectory);

float goal_distance_cost(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions, map<string, float> data);

double inefficiency_cost(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions, map<string, float> data);

double lane_speed(map<int, vector<Vehicle>> predictions, int lane, double s);

map<string, float> get_helper_data(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions);

float collision_cost(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions, map<string, float> data);

float buffer_cost(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions, map<string, float> data);

float logistic(float x);

float get_nearest_distance(vector<Vehicle> trajectory,map<int, vector<Vehicle>> predictions);

#endif /* cost_hpp */
