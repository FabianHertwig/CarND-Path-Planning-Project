//
// Created by Fabian on 03.06.18.
//

#ifndef PATH_PLANNING_PATH_H
#define PATH_PLANNING_PATH_H

#include <uWS/uWS.h>
#include <fstream>
//#include <opencl-c.h>
#include <cmath>
#include <valarray>
#include <complex>

#include <vector>
#include "utils.h"

using namespace std;

class Path {

    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;

public:
    Path(const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y);

    Path();

    const vector<double> &getMap_waypoints_x() const;

    const vector<double> &getMap_waypoints_y() const;

    void push_back(const double x_val, const double y_val);

    void shift_to_reference(double x_ref, double y_ref, double yaw_ref);

};


#endif //PATH_PLANNING_PATH_H
