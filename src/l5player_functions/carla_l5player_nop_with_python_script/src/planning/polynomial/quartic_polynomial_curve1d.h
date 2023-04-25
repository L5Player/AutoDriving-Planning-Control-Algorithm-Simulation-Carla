#ifndef QUARTIC_POLYNOMIAL_CURVE1D_H
#define QUARTIC_POLYNOMIAL_CURVE1D_H
#include "polynomial_curve1d.h"
#include "curve1d.h"
#include <vector>
#include <cmath>
#include <chrono>
#include <array>
#include <string>
#include <iostream>
//1D quartic polynomial curve : (x0, dx0, ddx0)--[ 0, param ]-- > (dx1, ddx1)
class QuarticPolynomialCurve1d : public PolynomialCurve1d
{
public:
    QuarticPolynomialCurve1d() = default;

    QuarticPolynomialCurve1d(const std::array<double, 3> &start,
                             const std::array<double, 2> &end,
                             const double param);

    QuarticPolynomialCurve1d(const double x0, const double dx0, const double ddx0,
                             const double dx1, const double ddx1,
                             const double param);

    QuarticPolynomialCurve1d(const QuarticPolynomialCurve1d &other);

    virtual ~QuarticPolynomialCurve1d() = default;

    double Evaluate(const std::uint32_t order, const double p) const override;

    /**
   * Interface with refine quartic polynomial by meets end first order
   * and start second order boundary condition:
   * @param  x0    init point x location
   * @param  dx0   init point derivative
   * @param  ddx0  init point second order derivative
   * @param  x1    end point x location
   * @param  dx1   end point derivative
   * @param  param parameter length
   * @return       self
   */
    QuarticPolynomialCurve1d &FitWithEndPointFirstOrder(
        const double x0, const double dx0, const double ddx0, const double x1,
        const double dx1, const double param);

    /**
   * Interface with refine quartic polynomial by meets end point second order
   * and start point first order boundary condition
   */
    QuarticPolynomialCurve1d &FitWithEndPointSecondOrder(
        const double x0, const double dx0, const double x1, const double dx1,
        const double ddx1, const double param);

    /*
   * Integrated from cubic curve with init value
   */
    QuarticPolynomialCurve1d &IntegratedFromCubicCurve(
        const PolynomialCurve1d &other, const double init_value);

    /*
   * Derived from quintic curve
   */
    QuarticPolynomialCurve1d &DerivedFromQuinticCurve(
        const PolynomialCurve1d &other);

    double ParamLength() const override { return param_; }

    std::string ToString() const override;

    double Coef(const size_t order) const override;

    size_t Order() const { return 4; }

private:
    void ComputeCoefficients(const double x0, const double dx0, const double ddx0,
                             const double dx1, const double ddx1,
                             const double param);

    std::array<double, 5> coef_ = {{0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 3> start_condition_ = {{0.0, 0.0, 0.0}};
    std::array<double, 2> end_condition_ = {{0.0, 0.0}};
};
#endif