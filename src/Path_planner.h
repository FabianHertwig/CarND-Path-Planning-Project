//
// Created by Fabian on 03.06.18.
//

#ifndef PATH_PLANNING_PATH_PLANNER_H
#define PATH_PLANNING_PATH_PLANNER_H

#include <math.h>
#include "Path.h"

using namespace std;

class Path_planner {


public:
    Path get_straight_path(double car_x, double car_y, double car_yaw);
    Path get_circular_path(double car_x, double car_y, double car_yaw, const Path &previous_path);
};


#endif //PATH_PLANNING_PATH_PLANNER_H
