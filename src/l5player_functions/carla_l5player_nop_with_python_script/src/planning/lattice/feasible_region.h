#pragma once
#include <algorithm>
#include "plan_init.h"
class FeasibleRegion
{
public:
    FeasibleRegion() = default;
    explicit FeasibleRegion(const std::array<double, 3> &init_s);
    // v = v0 + at
    // 表示自车在初始速度v0下，以设定的最大加速度行驶 t 秒之后的速度值
    double SUpper(const double t) const;

    double SLower(const double t) const;

    double VUpper(const double t) const;

    double VLower(const double t) const;

    double TLower(const double s) const;

private:
    std::array<double, 3> init_s_;

    double t_at_zero_speed_;

    double s_at_zero_speed_;

    ;
};