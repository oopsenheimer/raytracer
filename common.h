#ifndef COMMON_H
#define COMMON_H

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include<fstream>
#include "interval.h"

using std::fabs;
using std::make_shared;
using std::shared_ptr;
using std::sqrt;

const double infinity = std::numeric_limits<double>::infinity();
const double pi = M_PI;

inline double degrees_to_radians(double degrees) {
    return degrees * pi / 180.0;
}

inline double random_double() {
    
    return rand() / (RAND_MAX + 1.0);
}

inline double random_double(double min, double max) {
    
    return min + (max-min)*random_double();
}

#include "color.h"
#include "ray.h"
#include "vec3.h"

#endif
