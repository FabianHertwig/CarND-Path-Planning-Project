//
// Created by Fabian on 03.06.18.
//


#include "Path.h"

using namespace std;

Path::Path() {}

Path::Path(const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y) : map_waypoints_x(
        map_waypoints_x), map_waypoints_y(map_waypoints_y) {}

const vector<double> &Path::getMap_waypoints_x() const {
    return map_waypoints_x;
}

void Path::setMap_waypoints_x(const vector<double> &map_waypoints_x) {
    Path::map_waypoints_x = map_waypoints_x;
}

const vector<double> &Path::getMap_waypoints_y() const {
    return map_waypoints_y;
}

void Path::setMap_waypoints_y(const vector<double> &map_waypoints_y) {
    Path::map_waypoints_y = map_waypoints_y;
}

void Path::push_back(const double x_val, const double y_val) {
    map_waypoints_x.push_back(x_val);
    map_waypoints_y.push_back(y_val);
}

void Path::shift_to_reference(double x_ref, double y_ref, double yaw_ref) {

    for (int i = 0; i < map_waypoints_x.size(); ++i) {
        double shift_x = map_waypoints_x[i] - x_ref;
        double shift_y = map_waypoints_y[i] - y_ref;

        map_waypoints_x[i] = (shift_x * cos(0-yaw_ref) - shift_y*sin(0-yaw_ref));
        map_waypoints_y[i] = (shift_x * sin(0-yaw_ref) + shift_y*cos(0-yaw_ref));
    }
}

