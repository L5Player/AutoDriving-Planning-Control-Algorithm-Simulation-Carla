#pragma once
#include <cmath>
#include <algorithm>
#include <iostream>
#include <memory>
#include <vector>
#include "vec2d.h"
#include "path_points.h"
#include "trajectoryPoint.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

namespace common
{
  namespace math
  {
    /**
 * @brief Cross product between two 2-D vectors from the common start point,
 *        and end at two other points.
 * @param start_point The common start point of two vectors in 2-D.
 * @param end_point_1 The end point of the first vector.
 * @param end_point_2 The end point of the second vector.
 *
 * @return The cross product result.
 */
    double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1, const Vec2d &end_point_2);

    /**
 * @brief Cross product between two vectors.
 *        One vector is formed by 1st and 2nd parameters of the function.
 *        The other vector is formed by 3rd and 4th parameters of the function.
 * @param x0 The x coordinate of the first vector.
 * @param y0 The y coordinate of the first vector.
 * @param x1 The x coordinate of the second vector.
 * @param y1 The y coordinate of the second vector.
 *
 * @return The cross product result.
 */
    double CrossProd(const double x0, const double y0, const double x1, const double y1);

    /**
 * @brief Wrap angle to [0, 2 * PI).
 * @param angle the original value of the angle.
 * @return The wrapped value of the angle.
 */
    double WrapAngle(const double angle);

    /**
 * @brief Normalize angle to [-PI, PI).
 * @param angle the original value of the angle.
 * @return The normalized value of the angle.
 */
    double NormalizeAngle(const double angle);
    double Sigmoid(const double x);

    double slerp(const double a0, const double t0, const double a1, const double t1, const double t);

    // Cartesian coordinates to Polar coordinates
    std::pair<double, double> Cartesian2Polar(double x, double y);

    /**
 * @brief Linear interpolation between two points of type T.
 * @param x0 The coordinate of the first point.
 * @param t0 The interpolation parameter of the first point.
 * @param x1 The coordinate of the second point.
 * @param t1 The interpolation parameter of the second point.
 * @param t The interpolation parameter for interpolation.
 * @param x The coordinate of the interpolated point.
 * @return Interpolated point.
 */
    template <typename T>
    T lerp(const T &x0, const double t0, const T &x1, const double t1,
           const double t)
    {
      if (std::abs(t1 - t0) <= 1.0e-6)
      {
        // AERROR << "input time difference is too small";
        return x0;
      }
      const double r = (t - t0) / (t1 - t0);
      const T x = x0 + r * (x1 - x0);
      return x;
    }

    template <typename T>
    void uniform_slice(const T start, const T end, uint32_t num, std::vector<T> *sliced)
    {
      if (!sliced || num == 0)
      {
        return;
      }
      const T delta = (end - start) / num;
      sliced->resize(num + 1);
      T s = start;
      for (uint32_t i = 0; i < num; ++i, s += delta)
      {
        sliced->at(i) = s;
      }
      sliced->at(num) = end;
    }

    template <typename T>
    T Clamp(const T value, T bound1, T bound2)
    {
      if (bound1 > bound2)
      {
        std::swap(bound1, bound2);
      }

      if (value < bound1)
      {
        return bound1;
      }
      else if (value > bound2)
      {
        return bound2;
      }
      return value;
    }

    template <typename Container>
    typename Container::value_type MaxElement(const Container &elements)
    {
      return *std::max_element(elements.begin(), elements.end());
    }

    template <typename Container>
    typename Container::value_type MinElement(const Container &elements)
    {
      return *std::min_element(elements.begin(), elements.end());
    }

    template <typename T, int M, int N, typename D>
    void DenseToCSCMatrix(const Eigen::Matrix<T, M, N> &dense_matrix,
                          std::vector<T> *data, std::vector<D> *indices,
                          std::vector<D> *indptr)
    {
      constexpr double epsilon = 1e-9;
      int data_count = 0;
      for (int c = 0; c < dense_matrix.cols(); ++c)
      {
        indptr->emplace_back(data_count);
        for (int r = 0; r < dense_matrix.rows(); ++r)
        {
          if (std::fabs(dense_matrix(r, c)) < epsilon)
          {
            continue;
          }
          data->emplace_back(dense_matrix(r, c));
          ++data_count;
          indices->emplace_back(r);
        }
      }
      indptr->emplace_back(data_count);
    }
    SLPoint InterpolateUsingLinearApproximation(const SLPoint &p0, const SLPoint &p1, const double w);
    PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0, const PathPoint &p1, const double s);
    TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint &tp0, const TrajectoryPoint &tp1,
                                                        const double t);
  } // namespace math
} // namespace common
