//
// Created by Fabian on 03.06.18.
//

#include <cfloat>
#include "Path_planner.h"
#include "spline.h"

static const double KMH_TO_MS_FRACTION = 2.24;
using namespace std;

/**
 * Calculates a smooth path between waypoints like described in the class Q&A.
 * @param car_x
 * @param car_y
 * @param car_yaw
 * @param car_s
 * @param previous_path
 * @param map
 * @return
 */
Path Path_planner::get_smooth_path(double car_x, double car_y, double car_yaw, double car_s, const Path &previous_path,
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
    vector<double> next_wp0 = getXY(car_s + 50, (2 + 4 * current_lane), map);
    vector<double> next_wp1 = getXY(car_s + 100, (2 + 4 * current_lane), map);
    vector<double> next_wp2 = getXY(car_s + 150, (2 + 4 * current_lane), map);

    pts.push_back(next_wp0[0], next_wp0[1]);
    pts.push_back(next_wp1[0], next_wp1[1]);
    pts.push_back(next_wp2[0], next_wp2[1]);


    pts.shift_to_reference(ref_x, ref_y, ref_yaw);


    // Fit a spline
    tk::spline s;
    s.set_points(pts.getMap_waypoints_x(), pts.getMap_waypoints_y());

    Path next_vals;

    for (int j = 0; j < previous_path_size; ++j) {
        next_vals.push_back(previous_path_x[j], previous_path_y[j]);
    }

    // calculate points on the spline according to the set speed.
    double target_x = 50.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    double x_add_on = 0.0;

    for (int i = 1; i <= 50 - previous_path_size; ++i) {
        double n = (target_dist / (0.02 * current_speed / KMH_TO_MS_FRACTION));
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

/**
 * Sets the speed using a PID controller to keep a safety distance to the car in front.
 * If no car is in front it accelerates the car to the desired speed set in the path planner class.
 * @param car_s
 * @param sensor_fusion
 */
void Path_planner::set_speed(double car_s, vector<vector<double>> sensor_fusion) {

    bool is_car_in_front = is_other_car_in_front(car_s, current_lane, safety_distance + 10.0, sensor_fusion);
    if (is_car_in_front) {
        vector<double> closest_car = get_closest_car_in_front(car_s, current_lane, sensor_fusion);
        double other_car_s = closest_car[5];
        double distance = other_car_s - car_s;

        current_acceleration = distance_pid.calculate(distance, safety_distance);
        // std::cout << "current_acceleration: " << current_acceleration << std::endl;
    } else if (current_speed < desired_speed) {
        current_acceleration = acceleration;
    }

    if (current_speed > desired_speed) {
        current_acceleration = -acceleration;
    }

    current_speed += current_acceleration;
}

/**
 * Checks if another car in lane is closer then the distance parameter.
 * @param car_s
 * @param lane
 * @param distance
 * @param sensor_fusion
 * @return
 */
bool Path_planner::is_other_car_in_front(double car_s, int lane, double distance,
                                         const vector<vector<double>> &sensor_fusion) const {
    bool car_is_in_front = false;
    for (const auto &other_car : sensor_fusion) {
        double other_car_d = other_car[6];

        if (is_in_same_lane(other_car_d, lane)) {
            double other_car_s = other_car[5];

            if (other_car_s > car_s && (other_car_s - car_s) < distance) {
                car_is_in_front = true;
            }
        }
    }
    return car_is_in_front;
}

/**
 * Checks if another car is in the lane and in between distance to the front and distance / 3.0 to the rear of car_s.
 * @param car_s
 * @param lane
 * @param distance
 * @param sensor_fusion
 * @return
 */
bool Path_planner::is_other_car_close(double car_s, int lane, double distance,
                                      const vector<vector<double>> &sensor_fusion) const {
    bool car_is_close = false;
    for (const auto &other_car : sensor_fusion) {
        double other_car_d = other_car[6];

        if (is_in_same_lane(other_car_d, lane)) {
            double other_car_s = other_car[5];

            if (other_car_s > car_s && (other_car_s - car_s) < distance) {
                car_is_close = true;
            } else if (other_car_s < car_s && (car_s - other_car_s) < distance / 3.0) {
                car_is_close = true;
            }
        }
    }
    return car_is_close;
}

bool Path_planner::is_in_same_lane(double other_car_d, int lane) const {
    return other_car_d < (2 + 4 * lane + 2) && other_car_d > (2 + 4 * lane - 2);
}

/**
 * Sets current lane. Sqitches lane if there is a car in front and no car in the target lane.
 * @param car_s
 * @param sensor_fusion
 */
void Path_planner::consider_switch_lane(double car_s, const vector<vector<double>> &sensor_fusion) {

    double look_ahead_distance = safety_distance + safety_distance / 2.0;

    bool car_is_in_front = is_other_car_in_front(car_s, current_lane, look_ahead_distance, sensor_fusion);

    if (car_is_in_front) {
        if (current_lane == 2 || current_lane == 1) {
            if (!is_other_car_close(car_s, current_lane - 1, look_ahead_distance, sensor_fusion)) {
                current_lane = current_lane - 1;
                std::cout << "Switching to lane: " << current_lane << std::endl;
            }
        }
        if (current_lane == 0 || current_lane == 1) {
            if (!is_other_car_close(car_s, current_lane + 1, look_ahead_distance, sensor_fusion)) {
                current_lane = current_lane + 1;
                std::cout << "Switching to lane: " << current_lane << std::endl;
            }
        }
    }
}

Path_planner::Path_planner() {
    current_lane = 1;
    current_speed = 0.0;
    current_acceleration = 0.0;
}

vector<double> Path_planner::get_closest_car_in_front(double car_s, int lane, vector<vector<double>> sensor_fusion) {

    double distance = std::numeric_limits<double>::max();
    vector<double> closest_car;

    for (const auto &other_car : sensor_fusion) {
        double other_car_d = other_car[6];

        if (is_in_same_lane(other_car_d, lane)) {
            double other_car_s = other_car[5];

            double this_distance = other_car_s - car_s;
            if (this_distance > 0.0 && this_distance < distance) {
                distance = this_distance;
                closest_car = other_car;
            }
        }
    }

    return closest_car;
}


