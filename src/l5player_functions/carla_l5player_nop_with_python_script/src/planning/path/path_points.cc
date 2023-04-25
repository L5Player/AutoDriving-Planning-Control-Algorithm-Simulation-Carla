#include "path_points.h"

PathPoint::PathPoint() : is_x(false), is_y(false)
{
    x = 0;      // x position
    y = 0;      // y position
    z = 0;      // z position
    theta = 0;  // yaw in rad
    kappa = 0;  // curvature曲率
    dkappa = 0; // curvature曲率导数
    v = 0;      // Tangential velocity
    a = 0;      // Tangential acceleration
    s = 0;      // s position along spline
}

bool PathPoint::has_x() const
{
    return is_x;
}
bool PathPoint::has_y() const
{
    return is_y;
}
void PathPoint::set_x(const double x_)
{
    is_x = true;
    x = x_;
}
void PathPoint::set_y(const double y_)
{
    is_y = true;
    y = y_;
}
void PathPoint::set_z(const double z_)
{
    z = z_;
}
void PathPoint::set_s(const double s_)
{
    s = s_;
}
void PathPoint::set_theta(const double theta_)
{
    theta = theta_;
}
void PathPoint::set_kappa(const double kappa_)
{
    kappa = kappa_;
}
void PathPoint::set_dkappa(const double dkappa_)
{
    dkappa = dkappa_;
}
void PathPoint::set_v(const double v_)
{
    v = v_;
}
void PathPoint::set_a(const double a_)
{
    a = a_;
}

double PathPoint::get_x()
{
    return x;
}
double PathPoint::get_y()
{
    return y;
}

FrenetFramePoint::FrenetFramePoint() : is_s(false)
{

    s = 0;             // s position along spline
    relative_time = 0; // relative_time
    d = 0;             // lateral offset
    d_d = 0;           // lateral speed
    d_dd = 0;          // lateral acceleration
    d_ddd = 0;         // lateral jerk
    s_d = 0;           // s speed
    s_dd = 0;          // s acceleration
    s_ddd = 0;         // s jerk
}

bool FrenetFramePoint::has_s() const
{
    return is_s;
}
void FrenetFramePoint::set_s(const double s_)
{
    is_s = true;
    s = s_;
}
void FrenetFramePoint::set_ds(const double s_d_)
{
    s_d = s_d_;
}
void FrenetFramePoint::set_dds(const double s_dd_)
{
    s_dd = s_dd;
}
void FrenetFramePoint::set_l(const double d_)
{
    d = d_;
}
void FrenetFramePoint::set_dl(const double d_dd_)
{
    d_dd = d_dd_;
}
void FrenetFramePoint::set_ddl(const double d_ddd_)
{
    d_ddd = d_ddd_;
}

double FrenetFramePoint::get_s()
{
    return s;
}
double FrenetFramePoint::get_l()
{
    return d;
}

SpeedPoint::SpeedPoint() : is_s(false), is_v(false), is_t(false), is_a(false), is_da(false)
{
    s = 0; //
    v = 0;
    t = 0;
    a = 0;
    da = 0;
}
bool SpeedPoint::has_v() const
{
    return is_v;
}
bool SpeedPoint::has_a() const
{
    return is_a;
}
bool SpeedPoint::has_da() const
{
    return is_da;
}
bool SpeedPoint::has_s() const
{
    return is_s;
}
void SpeedPoint::set_s(const double s_)
{
    is_s = true;
    s = s_;
}
void SpeedPoint::set_t(const double t_)
{
    is_t = true;
    t = t_;
}
void SpeedPoint::set_v(const double v_)
{
    is_v = true;
    v = v_;
}
void SpeedPoint::set_a(const double a_)
{
    is_a = true;
    a = a_;
}
void SpeedPoint::set_da(const double da_) //加速度的导数
{
    is_da = true;
    da = da_;
}

void SpeedPoint::Clear()
{
    v = 0;
    t = 0;
    a = 0;
    da = 0;

    is_s = false;
    is_t = false;
    is_v = false;
    is_a = false;
    is_da = false;
}