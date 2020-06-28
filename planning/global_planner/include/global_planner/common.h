/*!
   \file
   \brief This is a collection of helper functions that are used throughout the project.

*/
#ifndef _COMMON_GLOBAL
#define _COMMON_GLOBAL

#include <cmath>
#include <algorithm>

#include "constance.h"
namespace planner {
/*!
    \brief The namespace that wraps helper.h
    \namespace Helper
*/
namespace common {

/*!
   \fn static inline double normalizeHeading(double t)
   \brief Normalizes a heading given in degrees to (0,360]
   \param t heading in degrees
*/
/**
 * 将朝向t的值归一化到[0, 360)，Degree
 */
static inline double normalizeHeading(double t) {
  if ((int)t <= 0 || (int)t >= 360) {
    if (t < -0.1) {
      t += 360.f;
    } else if ((int)t >= 360) {
      t -= 360.f;
    } else {
      t =  0;
    }
  }

  return t;
}

/*!
   \fn double normalizeHeadingRad(double t)
   \brief Normalizes a heading given in rad to (0,2PI]
   \param t heading in rad
*/
static inline double normalizeHeadingRad(double t) {
  if (t < 0) {
    t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
    return 2.f * M_PI + t;
  }

  return t - 2.f * M_PI * (int)(t / (2.f * M_PI));
}

/*!
   \fn double toDeg(double t)
   \brief Converts and normalizes a heading given in rad to deg
   \param t heading in deg
*/
static inline double toDeg(double t) {
  return normalizeHeadingRad(t) * 180.f / M_PI ;
}

/*!
   \fn double toRad(double t)
   \brief Converts and normalizes a heading given in deg to rad
   \param t heading in rad
*/
static inline double toRad(double t) {
  return normalizeHeadingRad(t / 180.f * M_PI);
}

/*!
   \fn double clamp(double n, double lower, double upper)
   \brief Clamps a number between a lower and an upper bound
   \param t heading in rad
*/
static inline double clamp(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

}
}

#endif 

