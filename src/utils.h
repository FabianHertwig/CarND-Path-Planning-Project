//
// Created by Fabian on 03.06.18.
//

#ifndef PATH_PLANNING_TOOLS_H
#define PATH_PLANNING_TOOLS_H

#include <math.h>

using namespace std;


// For converting back and forth between radians and degrees.
static double pi() { return M_PI; }

static double deg2rad(double x) { return x * pi() / 180; }

static double rad2deg(double x) { return x * 180 / pi(); }

#endif //PATH_PLANNING_TOOLS_H
