#include "trajectoryPoint.h"
#include "math_utils.h"

DiscretizedTrajectory::DiscretizedTrajectory(
    const std::vector<TrajectoryPoint> &trajectory_points)
    : std::vector<TrajectoryPoint>(trajectory_points)
{
    if (trajectory_points.empty())
        std::cout << "trajectory_points should NOT be empty()"
                  << "\n";
}

TrajectoryPoint DiscretizedTrajectory::Evaluate(
    const double relative_time) const
{
    auto comp = [](const TrajectoryPoint &p, const double relative_time)
    {
        return p.relative_time < relative_time;
    };

    auto it_lower = std::lower_bound(begin(), end(), relative_time, comp);

    if (it_lower == begin())
    {
        return front();
    }
    else if (it_lower == end())
    {
        //  std::cout << "When evaluate trajectory, relative_time(" << relative_time
        //       << ") is too large";
        return back();
    }
    return common::math::InterpolateUsingLinearApproximation(
        *(it_lower - 1), *it_lower, relative_time);
}

size_t DiscretizedTrajectory::QueryLowerBoundPoint(
    const double relative_time) const
{
    // CHECK(!empty());

    if (relative_time >= back().relative_time)
    {
        return size() - 1;
    }
    auto func = [](const TrajectoryPoint &tp, const double relative_time)
    {
        return tp.relative_time < relative_time;
    };
    auto it_lower = std::lower_bound(begin(), end(), relative_time, func);
    return std::distance(begin(), it_lower);
}

size_t DiscretizedTrajectory::QueryNearestPoint(
    const Vec2d &position) const
{
    double dist_sqr_min = std::numeric_limits<double>::max();
    size_t index_min = 0;
    for (size_t i = 0; i < size(); ++i)
    {
        const Vec2d curr_point(data()[i].path_point().x,
                               data()[i].path_point().y);

        const double dist_sqr = curr_point.DistanceSquareTo(position);
        if (dist_sqr < dist_sqr_min)
        {
            dist_sqr_min = dist_sqr;
            index_min = i;
        }
    }
    return index_min;
}

size_t DiscretizedTrajectory::QueryNearestPointWithBuffer(
    const Vec2d &position, const double buffer) const
{
    double dist_sqr_min = std::numeric_limits<double>::max();
    size_t index_min = 0;
    for (size_t i = 0; i < size(); ++i)
    {
        const Vec2d curr_point(data()[i].path_point().x,
                               data()[i].path_point().y);

        const double dist_sqr = curr_point.DistanceSquareTo(position);
        if (dist_sqr < dist_sqr_min + buffer)
        {
            dist_sqr_min = dist_sqr;
            index_min = i;
        }
    }
    return index_min;
}

void DiscretizedTrajectory::AppendTrajectoryPoint(
    const TrajectoryPoint &trajectory_point)
{
    if (!empty())
    {
        // CHECK_GT(trajectory_point.relative_time, back().relative_time);
    }
    push_back(trajectory_point);
}

const TrajectoryPoint &DiscretizedTrajectory::TrajectoryPointAt(
    const size_t index) const
{
    // CHECK_LT(index, NumOfPoints());
    return data()[index];
}

TrajectoryPoint DiscretizedTrajectory::StartPoint() const
{
    // CHECK(!empty());
    return front();
}

double DiscretizedTrajectory::GetTemporalLength() const
{
    // CHECK(!empty());
    return back().relative_time - front().relative_time;
}

double DiscretizedTrajectory::GetSpatialLength() const
{
    // CHECK(!empty());
    return back().path_point().s - front().path_point().s;
}

TrajectoryPoint::TrajectoryPoint()
{
    pathpoint_.x = 0;
    pathpoint_.y = 0;
    pathpoint_.z = 0;
    pathpoint_.v = 0;
    pathpoint_.a = 0;
    pathpoint_.s = 0;
    pathpoint_.theta = 0;
    pathpoint_.kappa = 0;
    pathpoint_.dkappa = 0;

    x = 0;             // x position
    y = 0;             // y position
    z = 0;             // z position
    theta = 0;         // yaw in rad
    kappa = 0;         // curvature曲率
    dkappa = 0;        // curvature曲率导数
    v = 0;             // Tangential velocity
    a = 0;             // Tangential acceleration
    relative_time = 0; // relative_time
    s = 0;             // s position along spline

    // 这些不是直接获取的
    // d = 0;      // lateral offset
    // d_d = 0;   // lateral speed
    // d_dd = 0; // lateral acceleration
    // s_d = 0;   // s speed
    // s_dd = 0; // s acceleration
    // s_ddd= 0;
    // d_ddd= 0;
    has_path_point_ = false;
}

TrajectoryPoint::TrajectoryPoint(const InitialConditions &Initial)
{
    pathpoint_.x = Initial.x_init;
    pathpoint_.y = Initial.y_init;
    pathpoint_.z = Initial.z_init;
    pathpoint_.v = Initial.v_init;
    pathpoint_.a = Initial.a_init;
    pathpoint_.s = Initial.s0;
    pathpoint_.theta = Initial.theta_init;
    pathpoint_.kappa = Initial.kappa_init;
    pathpoint_.dkappa = Initial.dkappa_init;

    x = Initial.x_init;                         // x position
    y = Initial.y_init;                         // y position
    z = Initial.z_init;                         // z position
    theta = Initial.theta_init;                 // yaw in rad
    kappa = Initial.kappa_init;                 // curvature曲率
    dkappa = Initial.dkappa_init;               // curvature曲率导数
    v = Initial.v_init;                         // Tangential velocity
    a = Initial.a_init;                         // Tangential acceleration
    relative_time = Initial.init_relative_time; // relative_time
    s = Initial.s0;                             // s position along spline

    // 这些不是直接获取的
    // d = Initial.d0;      // lateral offset
    // d_d = Initial.dd0;   // lateral speed
    // d_dd = Initial.ddd0; // lateral acceleration
    // s_d = Initial.ds0;   // s speed
    // s_dd = Initial.dds0; // s acceleration
    // s_ddd= 0;
    // d_ddd= 0;
    
    has_path_point_ = false;
}

PathPoint TrajectoryPoint::path_point() const
{
    return pathpoint_;
}
bool TrajectoryPoint::has_path_point() const
{
    return has_path_point_;
}
void TrajectoryPoint::CopyFrom(PathPoint &path_point, FrenetFramePoint &frenet_point)
{
    x = path_point.x;           // x position
    y = path_point.y;           // y position
    z = path_point.z;           // z position
    theta = path_point.theta;   // yaw in rad
    kappa = path_point.kappa;   // curvature曲率
    dkappa = path_point.dkappa; // curvature曲率导数
    v = path_point.v;           // Tangential velocity
    a = path_point.a;           // Tangential acceleration

    d = frenet_point.d;       // lateral offset
    d_d = frenet_point.d_d;   // lateral speed
    d_dd = frenet_point.d_dd; // lateral acceleration
    s = path_point.s;         // s position along spline
    s_d = frenet_point.s_d;   // s speed
    s_dd = frenet_point.s_dd; // s acceleration

    pathpoint_ = std::move(path_point);
}

void TrajectoryPoint::set_x(double x_)
{
    has_path_point_ = true;
    x = x_;
    pathpoint_.set_x(x_);
}
void TrajectoryPoint::set_y(double y_)
{
    y = y_;
    pathpoint_.set_y(y_);
}
void TrajectoryPoint::set_z(double z_)
{
    z = z_;
    pathpoint_.set_z(z_);
}
void TrajectoryPoint::set_theta(double theta_)
{
    theta = theta_;
    pathpoint_.set_theta(theta_);
}
void TrajectoryPoint::set_s(double s_)
{
    s = s_;
    pathpoint_.set_s(s_);
}
void TrajectoryPoint::set_kappa(double kappa_)
{
    kappa = kappa_;
    pathpoint_.set_kappa(kappa_);
}
void TrajectoryPoint::set_dkappa(double dkappa_)
{
    dkappa = dkappa_;
    pathpoint_.set_dkappa(dkappa_);
}
void TrajectoryPoint::set_v(double v_)
{
    v = v_;
    pathpoint_.set_v(v_);
}
void TrajectoryPoint::set_a(double a_)
{
    a = a_;
    pathpoint_.set_a(a_);
}
void TrajectoryPoint::set_relative_time(double relative_time_)
{
    relative_time = relative_time_;
}

void TrajectoryPoint::set_absolute_time(double absolute_time_)
{
    absolute_time = absolute_time_;
}