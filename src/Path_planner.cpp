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

Path Path_planner::get_circular_path(double car_x, double car_y, double car_yaw, const Path &previous_path) {
    double pos_x;
    double pos_y;
    double angle;

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    vector<double> previous_path_x = previous_path.getMap_waypoints_x();
    vector<double> previous_path_y = previous_path.getMap_waypoints_y();

    int path_size = previous_path_x.size();

    for(int i = 0; i < path_size; i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    if(path_size == 0)
    {
        pos_x = car_x;
        pos_y = car_y;
        angle = deg2rad(car_yaw);
    }
    else
    {
        pos_x = previous_path_x[path_size-1];
        pos_y = previous_path_y[path_size-1];

        double pos_x2 = previous_path_x[path_size-2];
        double pos_y2 = previous_path_y[path_size-2];
        angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
    }

    double dist_inc = 0.5;
    for(int i = 0; i < 50-path_size; i++)
    {
        next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
        next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
        pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
        pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
    }

    return Path(next_x_vals, next_y_vals);
}


