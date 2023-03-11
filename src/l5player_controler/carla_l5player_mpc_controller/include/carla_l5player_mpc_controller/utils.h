#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <string>
#include <vector>
#include <limits>

// for convenience
using std::string;
using std::vector;

const double lane_width		= 4.0;		// width of a lane					(m)
const double safety_margin	= 20.0;		// distance to keep from other cars	(m)
const double max_safe_speed	= 49.5;		// max reference speed in the limit	(mph)

/* *****************************************************************************************************************
- FunctionName: 
- Function    : 
- Inputs      : 
- Outputs     : 
- Comments    : 对于修饰Object来说，const并未区分出编译期常量和运行期常量, constexpr限定在了编译期常量 
***************************************************************************************************************** */
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

/* *****************************************************************************************************************
- FunctionName: 
- Function    : Calculate distance between two points
- Inputs      : 
- Outputs     : 
- Comments    : 
***************************************************************************************************************** */
double distance_two_point(double x1, double y1, double x2, double y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

/* *****************************************************************************************************************
- FunctionName: 
- Function    : Check if a vehicle is is a certain lane
- Inputs      : 
- Outputs     : 
- Comments    : 
***************************************************************************************************************** */
bool is_in_lane(double d, int lane) 
{
	return (d > lane_width * lane) && (d < lane_width * lane + lane_width);
}

#endif