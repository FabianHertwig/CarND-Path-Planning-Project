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

class path {

    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;

public:
    path(const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y);

    const vector<double> &getMap_waypoints_x() const;

    void setMap_waypoints_x(const vector<double> &map_waypoints_x);

    const vector<double> &getMap_waypoints_y() const;

    void setMap_waypoints_y(const vector<double> &map_waypoints_y);

};


#endif //PATH_PLANNING_PATH_H
