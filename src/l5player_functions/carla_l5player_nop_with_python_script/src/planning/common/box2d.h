#pragma once
#include "vec2d.h"
#include <vector>
#include <iostream>
#include <limits>
#include <string>
#include "line_segment2d.h"

class Box2d
{
public:
  Box2d() = default;
  /**
   * @brief Constructor which takes the center, heading, length and width.
   * @param center The center of the rectangular bounding box.
   * @param heading The angle between the x-axis and the heading-axis,
   *        measured counter-clockwise.
   * @param length The size of the heading-axis.
   * @param width The size of the axis perpendicular to the heading-axis.
   */
  Box2d(const Vec2d &center, const double heading, const double length, const double width);
  /**
   * @brief Tests points for membership in the box
   * @param point A point that we wish to test for membership in the box
   * @return True iff the point is contained in the box
   */
  bool IsPointIn(const Vec2d &point) const;
  /**
   * @brief Shifts this box by a given vector
   * @param shift_vec The vector determining the shift
   */

  void Shift(const Vec2d &shift_vec);
  /**
   * @brief Determines whether this box overlaps a given line segment
   * @param line_segment The line-segment
   * @return True if they overlap
   */
  bool HasOverlap(const LineSegment2d &line_segment) const;
  /**
   * @brief Determines whether these two boxes overlap
   * @param line_segment The other box
   * @return True if they overlap
   */
  bool HasOverlap(const Box2d &box) const;

  /**
   * @brief Determines the distance between the box and a given point
   * @param point The point whose distance to the box we wish to compute
   * @return A distance
   */
  double DistanceTo(const Vec2d &point) const;
  /**
   * @brief Determines the distance between the box and a given line segment
   * @param line_segment The line segment whose distance to the box we compute
   * @return A distance
   */
  double DistanceTo(const LineSegment2d &line_segment) const;
  /**
   * @brief Determines the distance between two boxes
   * @param box The box whose distance to this box we want to compute
   * @return A distance
   */
  double DistanceTo(const Box2d &box) const;
  /**
   * @brief Extend the box longitudinally
   * @param extension_length the length to extend
   */
  void LongitudinalExtend(const double extension_length);

  void LateralExtend(const double extension_length);
  /**
   * @brief Getter of the corners of the box
   * @param corners The vector where the corners are listed
   */
  void GetAllCorners(std::vector<Vec2d> *const corners) const;
  /**
   * @brief Getter of the corners of the box
   * @param corners The vector where the corners are listed
   */
  std::vector<Vec2d> GetAllCorners() const;

  void InitCorners();

  /**
   * @brief Getter of the center of the box
   * @return The center of the box
   */
  const Vec2d &center() const
  {
    return center_;
  }

  /**
   * @brief Getter of the x-coordinate of the center of the box
   * @return The x-coordinate of the center of the box
   */
  double center_x() const
  {
    return center_.x();
  }

  /**
   * @brief Getter of the y-coordinate of the center of the box
   * @return The y-coordinate of the center of the box
   */
  double center_y() const
  {
    return center_.y();
  }

  /**
   * @brief Getter of the length
   * @return The length of the heading-axis
   */
  double length() const
  {
    return length_;
  }

  /**
   * @brief Getter of the width
   * @return The width of the box taken perpendicularly to the heading
   */
  double width() const
  {
    return width_;
  }

  /**
   * @brief Getter of half the length
   * @return Half the length of the heading-axis
   */
  double half_length() const
  {
    return half_length_;
  }

  /**
   * @brief Getter of half the width
   * @return Half the width of the box taken perpendicularly to the heading
   */
  double half_width() const
  {
    return half_width_;
  }

  /**
   * @brief Getter of the heading
   * @return The counter-clockwise angle between the x-axis and the heading-axis
   */
  double heading() const
  {
    return heading_;
  }

  /**
   * @brief Getter of the cosine of the heading
   * @return The cosine of the heading
   */
  double cos_heading() const
  {
    return cos_heading_;
  }

  /**
   * @brief Getter of the sine of the heading
   * @return The sine of the heading
   */
  double sin_heading() const
  {
    return sin_heading_;
  }

  /**
   * @brief Getter of the area of the box
   * @return The product of its length and width
   */
  double area() const
  {
    return length_ * width_;
  }
  double max_x() const
  {
    return max_x_;
  }
  double min_x() const
  {
    return min_x_;
  }
  double max_y() const
  {
    return max_y_;
  }
  double min_y() const
  {
    return min_y_;
  }

private:
  Vec2d center_;
  double length_ = 0.0;
  double width_ = 0.0;
  double half_length_ = 0.0;
  double half_width_ = 0.0;
  double heading_ = 0.0;
  double cos_heading_ = 1.0;
  double sin_heading_ = 0.0;

  std::vector<Vec2d> corners_;

  double max_x_ = std::numeric_limits<double>::lowest();
  double min_x_ = std::numeric_limits<double>::max();
  double max_y_ = std::numeric_limits<double>::lowest();
  double min_y_ = std::numeric_limits<double>::max();
};