#ifndef POLYNOMIAL_CURVE1D_H
#define POLYNOMIAL_CURVE1D_H
#include "curve1d.h"
class PolynomialCurve1d : public Curve1d
{
public:
    PolynomialCurve1d() = default;
    virtual ~PolynomialCurve1d() = default;

    virtual double Coef(const size_t order) const = 0;
    virtual size_t Order() const = 0;

protected:
    double param_ = 0.0;
};
#endif