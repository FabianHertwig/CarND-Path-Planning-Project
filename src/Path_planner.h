//
// Created by Fabian on 03.06.18.
//

#ifndef PATH_PLANNING_PATH_PLANNER_H
#define PATH_PLANNING_PATH_PLANNER_H

#include <math.h>
#include "Path.h"
#include "Map.h"
#include "pid.h"


using namespace std;

class Path_planner {


    int current_lane;
    double current_speed;
    double current_acceleration;

    double safety_distance = 30.0;
    double desired_speed = 49.0;
    double acceleration = 0.5;

    PID distance_pid = PID(1.0 / 20.0, acceleration, -acceleration, 0.03, 0.1, 0.0);


public:
    Path_planner();


    Path get_smooth_path(double car_x, double car_y, double car_yaw, double car_s, const Path &previous_path,
                             const Map &map);


    bool is_in_same_lane(double other_car_d, int lane) const;

    void set_speed(double car_s, vector<vector<double>> sensor_fusion);

    void consider_switch_lane(double car_s, const vector<vector<double>> &sensor_fusion);

    vector<double> get_closest_car_in_front(double car_s, int lane, vector<vector<double>> sensor_fusion);

    bool
    is_other_car_in_front(double car_s, int lane, double distance, const vector<vector<double>> &sensor_fusion) const;

    bool is_other_car_close(double car_s, int lane, double distance, const vector<vector<double>> &sensor_fusion) const;
};


#endif //PATH_PLANNING_PATH_PLANNER_H
