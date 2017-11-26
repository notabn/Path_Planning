//
//  cost.cpp
//  Behavioural Planner
//
//  Copyright © 2017 Natalia Balcu. All rights reserved.
//

#include "cost.hpp"
#include "vehicle.hpp"

#include <functional>
#include <iterator>
#include <map>
#include <math.h>


//TODO: change weights for cost functions.
const float REACH_GOAL = pow(10,3);
const float EFFICIENCY = pow(10,6);
const float COLLISION = pow(10,7);
const float VEHICLE_RADIUS = 1.5;
const float BUFFER =  0;
const float JERK =  0;
const float ACC =  0;

/*
 Here we have provided two possible suggestions for cost functions, but feel free to use your own!
 The weighted cost over all cost functions is computed in calculate_cost. See get_helper_data
 for details on how useful helper data is computed.
 */

float logistic(float x){
    /*
     A function that returns a value between 0 and 1 for x in the
     range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
     Useful for cost functions.
     */
    
    return 2.0 / (1 + exp(-x)) - 1.0;
    
}

float max_accel_cost(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions, map<string, float> data){
    int a = trajectory[1].a;
    if (abs(a) > vehicle.MAX_ACCEL){
        return 1.0;
    }else{
        return 0.0;
    }
}

float max_jerk_cost(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions, map<string, float> data){

    float max_jerk = (trajectory[1].a-trajectory[0].a)/trajectory[0].dt;
    
    if (max_jerk > vehicle.MAX_JERK){
        return 1.0;
    }else{
        return 0.0;
    }
}



float collision_cost(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions, map<string, float> data){
    /*
    Binary cost function which penalizes collisions.
    */
    float nearest = vehicle.get_nearest_distance(trajectory,predictions);
    //cout<<" nearest "<<nearest<<endl;
    if (nearest < 20){
        cout<<"<!!!!!!!!!! collision"<<endl;
        return 1.0;
    }else{
        return 0.0;
    }
    
}
float buffer_cost(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions, map<string, float> data){
    /*
    Penalizes getting close to other vehicles.
    */
    float nearest = vehicle.get_nearest_distance(trajectory,predictions);
    //cout<<"nearest "<<nearest<<endl;
    return logistic(2*VEHICLE_RADIUS / nearest);
    
}



float goal_distance_cost(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions, map<string, float> data) {
    /*
     Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
     Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
     */
    
   
    float cost;
    float distance = data["distance_to_goal"];
    if (distance > 0) {
        cost = 1 - 2*exp(-(trajectory[1].lane +trajectory[0].lane - data["intended_lane"] - data["final_lane"]) / distance);
    } else {
        cost = 1;
    }
    
    
    return cost;

}

float inefficiency_cost(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions, map<string, float> data) {
    /*
     Cost becomes higher for trajectories with intended lane and final lane that have slower traffic.
     */
    //cout<<"intended lane "<<data["intended_lane"]<<" final lane "<<data["final_lane"]<<endl;
    float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
    
    //cout<<"speed_intended "<<proposed_speed_intended<<endl;
    if (proposed_speed_intended <0){
         proposed_speed_intended = vehicle.target_speed;
    }
    
    float proposed_speed_final = lane_speed(predictions,data["final_lane"]);
    //cout<<"speed_final "<<proposed_speed_final<<endl;
    if (proposed_speed_final <0){
        proposed_speed_final = vehicle.target_speed;
    }
   
    float cost = (2*vehicle.target_speed - proposed_speed_intended-proposed_speed_final)/vehicle.target_speed;
    
    return cost;
}

float lane_speed(map<int, vector<Vehicle>> predictions, int lane) {
    /*
     All non ego vehicles in a lane have the same speed, so to get the speed limit for a lane,
     we can just find one vehicle in that lane.
     */
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        int key = it->first;
        Vehicle vehicle = it->second[0];
        
        if (vehicle.lane == lane && key != -1) {
            //cout<<"id "<<key<<"lane "<<vehicle.lane<<" speed "<<vehicle.v<<endl;
            return vehicle.v;
        }
    }
    //Found no vehicle in the lane
    return -1.0;
}

float calculate_cost(Vehicle vehicle, map<int, vector<Vehicle>> predictions, vector<Vehicle> trajectory) {
    /*
     Sum weighted cost functions to get total cost for trajectory.
     */
    map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
    float cost = 0.0;
    
    //Add additional cost functions here.
    vector< function<float(Vehicle, vector<Vehicle>, map<int, vector<Vehicle>>, map<string, float>)>> cf_list = {inefficiency_cost,goal_distance_cost,collision_cost,buffer_cost,max_accel_cost,max_jerk_cost};
    vector<float> weight_list = {EFFICIENCY,REACH_GOAL,COLLISION,BUFFER,ACC,JERK};
    
    for (int i = 0; i < cf_list.size(); i++) {
        float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, trajectory_data);
        cost += new_cost;
    }
    //cout <<"cost is"<<cost;
    return cost;
    
}

map<string, float> get_helper_data(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions) {
    /*
     Generate helper data to use in cost functions:
     intended_lane: +/- 1 from the current lane if the vehicle is planning or executing a lane change.
     final_lane: The lane of the vehicle at the end of the trajectory. The lane is unchanged for KL and PLCL/PLCR trajectories.
     distance_to_goal: The s distance of the vehicle to the goal.
     
     Note that indended_lane and final_lane are both included to help differentiate between planning and executing
     a lane change in the cost functions.
     */
    map<string, float> trajectory_data;
    Vehicle trajectory_last = trajectory[1];
    float intended_lane;
    
    
    if (trajectory_last.state.compare("PLCL") == 0) {
        intended_lane = trajectory_last.lane + 1;
    } else if (trajectory_last.state.compare("PLCR") == 0) {
        intended_lane = trajectory_last.lane - 1;
    } else {
        intended_lane = trajectory_last.lane;
    }

    float distance_to_goal = vehicle.goal_s - trajectory_last.s;
    float final_lane = trajectory_last.lane;
    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;
    trajectory_data["distance_to_goal"] = distance_to_goal;
    //cout<<"intended lane "<<intended_lane<<" final lane"<<final_lane<<endl;
    
    return trajectory_data;
}


