//
// Created by Fabian on 03.06.18.
//

#include "Path_planner.h"

using namespace std;


Path Path_planner::get_straight_path(double car_x, double car_y, double car_yaw) {
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    double dist_inc = 0.5;
    for (int i = 0; i < 50; i++) {
        next_x_vals.push_back(car_x + (dist_inc * i) * cos(deg2rad(car_yaw)));
        next_y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));
    }

    return Path(next_x_vals, next_y_vals);
}
