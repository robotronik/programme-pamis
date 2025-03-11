#pragma once
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.1415926
#endif

namespace angles {


static inline double from_degrees(double degrees) {
  return degrees * M_PI / 180.0;
}


static inline double to_degrees(double radians) {
  return radians * 180.0 / M_PI;
}


static inline double normalize_angle_positive(double angle) {
  return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}


static inline double normalize_angle(double angle) {
  double a = normalize_angle_positive(angle);

  if (a > M_PI) {
    a -= 2.0 * M_PI;
  }

  return a;
}

}