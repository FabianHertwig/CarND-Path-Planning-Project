//
// Created by Fabian on 03.06.18.
//

#include <sstream>
#include "Map.h"

Map::Map() {

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

}

const vector<double> &Map::getMap_waypoints_x() const {
    return map_waypoints_x;
}

const vector<double> &Map::getMap_waypoints_y() const {
    return map_waypoints_y;
}

const vector<double> &Map::getMap_waypoints_s() const {
    return map_waypoints_s;
}

