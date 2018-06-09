//
// Created by Fabian on 03.06.18.
//

#include "Path_planner.h"
#include "spline.h"

static const double KMH_TO_MS_FRACTION = 2.24;
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


Path Path_planner::get_stay_in_lane_path(double car_s, double car_d, Map map) {
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    vector<double> next_s_vals;
    vector<double> next_d_vals;

    double dist_inc = 0.5;
    for (int i = 0; i < 50; i++) {
        double s = car_s + (dist_inc * (i + 1));
        vector<double> xy = getXY(s, car_d, map);
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
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

    for (int i = 0; i < path_size; i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    if (path_size == 0) {
        pos_x = car_x;
        pos_y = car_y;
        angle = deg2rad(car_yaw);
    } else {
        pos_x = previous_path_x[path_size - 1];
        pos_y = previous_path_y[path_size - 1];

        double pos_x2 = previous_path_x[path_size - 2];
        double pos_y2 = previous_path_y[path_size - 2];
        angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
    }

    double dist_inc = 0.5;
    for (int i = 0; i < 50 - path_size; i++) {
        next_x_vals.push_back(pos_x + (dist_inc) * cos(angle + (i + 1) * (pi() / 100)));
        next_y_vals.push_back(pos_y + (dist_inc) * sin(angle + (i + 1) * (pi() / 100)));
        pos_x += (dist_inc) * cos(angle + (i + 1) * (pi() / 100));
        pos_y += (dist_inc) * sin(angle + (i + 1) * (pi() / 100));
    }

    return Path(next_x_vals, next_y_vals);
}


Path Path_planner::get_stay_in_lane_path_smooth(double car_x, double car_y, double car_yaw, double car_s, double car_d,
                                                int lane, const double ref_velocity, const Path &previous_path,
                                                const Map &map) {

    Path pts;

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    vector<double> previous_path_x = previous_path.getMap_waypoints_x();
    vector<double> previous_path_y = previous_path.getMap_waypoints_y();

    int previous_path_size = previous_path_x.size();

    // Set starting reference
    if (previous_path_size < 2) {
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        pts.push_back(prev_car_x, prev_car_y);
        pts.push_back(car_x, car_y);

    } else {
        ref_x = previous_path_x[previous_path_size - 1];
        ref_y = previous_path_y[previous_path_size - 1];

        double ref_x_prev = previous_path_x[previous_path_size - 2];
        double ref_y_prev = previous_path_y[previous_path_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        pts.push_back(ref_x_prev, ref_y_prev);
        pts.push_back(ref_x, ref_y);
    }

    // Create 30m spaced points
    vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map);
    vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map);
    vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map);

    pts.push_back(next_wp0[0], next_wp0[1]);
    pts.push_back(next_wp1[0], next_wp1[1]);
    pts.push_back(next_wp2[0], next_wp2[1]);


    pts.shift_to_reference(ref_x, ref_y, ref_yaw);

    tk::spline s;
    s.set_points(pts.getMap_waypoints_x(), pts.getMap_waypoints_y());

    Path next_vals;

    for (int j = 0; j < previous_path_size; ++j) {
        next_vals.push_back(previous_path_x[j], previous_path_y[j]);
    }

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    double x_add_on = 0.0;

    for (int i = 1; i <= 50 - previous_path_size; ++i) {
        double n = (target_dist / (0.02 * ref_velocity / KMH_TO_MS_FRACTION));
        double x_point = x_add_on + target_x / n;
        double y_point = s(x_point);

        x_add_on = x_point;


        double x_ref = x_point;
        double y_ref = y_point;

        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_vals.push_back(x_point, y_point);
    }

    return next_vals;

}


