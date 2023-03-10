#include "utils.h"

/* *****************************************************************************************************************
- FunctionName: 
- Function    : Calculate closest waypoint to current x, y position
- Inputs      : 
- Outputs     : 
- Comments    : 
***************************************************************************************************************** */
int get_closest_waypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{
    double closest_len = std::numeric_limits<int>::max();
    int closest_waypoint = 0;

    for (uint i = 0; i < maps_x.size(); ++i)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance_two_point(x, y, map_x, map_y);
        if (dist < closest_len)
        {
            closest_len = dist;
            closest_waypoint = i;
        }
    }
    return closest_waypoint;
}

/* *****************************************************************************************************************
- FunctionName: 
- Function    : Choose in the map of highway waypoints the closest before the car (that is the next).
                The actual closest waypoint could be behind the car.
- Inputs      : 
- Outputs     : 
- Comments    : 先找到距离最近的点，再根据车辆的航向与车辆当前位置点和最近点的连线方向形成的夹角的范围确定是不是前方的最近的一个，如果不是，就将索引加1
***************************************************************************************************************** */
int get_next_waypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    uint closest_waypoint = get_closest_waypoint(x, y, maps_x, maps_y);

    double map_x = maps_x[closest_waypoint];
    double map_y = maps_y[closest_waypoint];

    double heading = atan2((map_y - y), (map_x - x));

    double angle = fabs(theta - heading);
    angle = std::min(2 * pi() - angle, angle);

    if (angle > pi() / 2)
    {
        ++closest_waypoint;
        if (closest_waypoint == maps_x.size())
        {
            closest_waypoint = 0;
        }
    }

    return closest_waypoint;
}