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


public:
    Path get_straight_path(double car_x, double car_y, double car_yaw);
    Path get_circular_path(double car_x, double car_y, double car_yaw, const Path &previous_path);
    Path get_stay_in_lane_path(double car_s, double car_d, Map map);
};


#endif //PATH_PLANNING_PATH_PLANNER_H
