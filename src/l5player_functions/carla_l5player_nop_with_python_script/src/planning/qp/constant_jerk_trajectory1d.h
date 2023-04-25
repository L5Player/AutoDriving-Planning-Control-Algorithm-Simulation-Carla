#pragma once
#include <string>
#include "curve1d.h"
#include "plan_init.h"
class ConstantJerkTrajectory1d : public Curve1d
{
public:
    ConstantJerkTrajectory1d(const double p0, const double v0, const double a0,
                             const double jerk, const double param);

    virtual ~ConstantJerkTrajectory1d() = default;

    double Evaluate(const std::uint32_t order, const double param) const;

    double ParamLength() const;

    std::string ToString() const;

    double start_position() const;

    double start_velocity() const;

    double start_acceleration() const;

    double end_position() const;

    double end_velocity() const;

    double end_acceleration() const;

    double jerk() const;

private:
    double p0_;
    double v0_;
    double a0_;

    double p1_;
    double v1_;
    double a1_;

    double param_;

    double jerk_;
};