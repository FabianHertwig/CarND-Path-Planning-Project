//
// Created by Fabian on 03.06.18.
//


#include "Path.h"

using namespace std;


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
