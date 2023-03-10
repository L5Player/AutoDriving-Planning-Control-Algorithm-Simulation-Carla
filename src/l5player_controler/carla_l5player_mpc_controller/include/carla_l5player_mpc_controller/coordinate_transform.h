#include <math.h>
#include <string>
#include <vector>
#include "waypoints.h"

// for convenience
using std::string;
using std::vector;

/* *****************************************************************************************************************
- FunctionName: 
- Function    : Transform from Cartesian x,y coordinates to Frenet s,d coordinates
- Inputs      : 
- Outputs     : 
- Comments    : 
***************************************************************************************************************** */
vector<double> cartesian_to_frenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int next_wp = get_next_waypoint(x, y, theta, maps_x, maps_y);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0)
    {
        prev_wp = maps_x.size() - 1;
    }

    double n_x = maps_x[next_wp] - maps_x[prev_wp];
    double n_y = maps_y[next_wp] - maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance_two_point(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point
    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double centerToPos = distance_two_point(center_x, center_y, x_x, x_y);
    double centerToRef = distance_two_point(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; ++i)
    {
        frenet_s += distance_two_point(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
    }

    frenet_s += distance_two_point(0, 0, proj_x, proj_y);

    return {frenet_s, -frenet_d};
}

/* *****************************************************************************************************************
- FunctionName: 
- Function    : Transform from Frenet s,d coordinates to Cartesian x,y
- Inputs      : 
- Outputs     : 
- Comments    : 
***************************************************************************************************************** */
vector<double> frenet_to_cartesian(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
    {
        ++prev_wp;
    }

    int wp2 = (prev_wp + 1) % maps_x.size();

    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));

    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};
}
