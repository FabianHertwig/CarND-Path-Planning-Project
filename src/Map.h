//
// Created by Fabian on 03.06.18.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <fstream>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>


using namespace std;

class Map {

    string map_file_ = "../data/highway_map.csv";
    double max_s = 6945.554;

    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

public:
    Map();

    double getMax_s() const;

    const vector<double> &getMap_waypoints_x() const;

    const vector<double> &getMap_waypoints_y() const;

    const vector<double> &getMap_waypoints_s() const;

    const vector<double> &getMap_waypoints_dx() const;

    const vector<double> &getMap_waypoints_dy() const;
};


#endif //PATH_PLANNING_MAP_H
