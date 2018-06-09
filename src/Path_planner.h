//
// Created by Fabian on 03.06.18.
//

#ifndef PATH_PLANNING_PATH_PLANNER_H
#define PATH_PLANNING_PATH_PLANNER_H

#include <math.h>
#include "Path.h"
#include "Map.h"


using namespace std;

class Path_planner {

    int current_lane;
    double current_speed;
    double current_acceleration;

    double safety_distance = 30;
    double desired_speed = 49.0;
    double acceleration = 0.3;




public:
    Path_planner(int current_lane, double desired_speed, double current_speed, double acceleration);


public:
    Path get_straight_path(double car_x, double car_y, double car_yaw);
    Path get_circular_path(double car_x, double car_y, double car_yaw, const Path &previous_path);
    Path get_stay_in_lane_path(double car_s, double car_d, Map map);

    Path get_stay_in_lane_path_smooth(double car_x, double car_y, double car_yaw, double car_s, double car_d,
                                      const Path &previous_path, const Map &map);


    bool is_in_same_lane(double other_car_d, int lane) const;

    bool is_other_car_in_front(double car_s, int lane, const vector<vector<double>> &sensor_fusion) const;

    void set_speed(double end_path_s, double car_s, double car_d, vector<vector<double>> sensor_fusion);
};


#endif //PATH_PLANNING_PATH_PLANNER_H
