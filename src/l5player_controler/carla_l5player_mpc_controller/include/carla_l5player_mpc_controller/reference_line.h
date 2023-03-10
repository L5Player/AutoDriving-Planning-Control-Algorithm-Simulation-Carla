#ifndef REFERENCE_LINE_H_
#define REFERENCE_LINE_H_
#include <math.h>

#include <iostream>
#include <vector>

namespace l5player {
namespace control {
class ReferenceLine {
public:
  ReferenceLine(const std::vector<std::pair<double, double>> &xy_points);
  ~ReferenceLine() = default;

  bool ComputePathProfile(std::vector<double> *headings,
                          std::vector<double> *accumulated_s,
                          std::vector<double> *kappas,
                          std::vector<double> *dkappas);

private:
  std::vector<std::pair<double, double>> xy_points_;
};

} // namespace control
} // namespace l5player
#endif