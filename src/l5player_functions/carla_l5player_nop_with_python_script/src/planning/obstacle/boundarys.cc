#include "boundarys.h"
#include "math_utils.h"

ST_Boundary::ST_Boundary(const std::vector<std::pair<STPoint, STPoint>> &point_pairs, bool is_accurate_boundary)
{

  std::vector<std::pair<STPoint, STPoint>> reduced_pairs(point_pairs);
  if (!is_accurate_boundary)
  {
    RemoveRedundantPoints(&reduced_pairs);
  }

  for (const auto &item : reduced_pairs)
  {
    // use same t for both points
    const double t = item.first.t();
    lower_points_.emplace_back(item.first.s(), t);
    upper_points_.emplace_back(item.second.s(), t);
  }
}

bool ST_Boundary::IsPointNear(const LineSegment2d &seg, const Vec2d &point, const double max_dist)
{
  return seg.DistanceSquareTo(point) < max_dist * max_dist;
}

void ST_Boundary::RemoveRedundantPoints(std::vector<std::pair<STPoint, STPoint>> *point_pairs)
{
  if (!point_pairs || point_pairs->size() <= 2)
  {
    return;
  }

  const double kMaxDist = 0.1;
  size_t i = 0;
  size_t j = 1;

  while (i < point_pairs->size() && j + 1 < point_pairs->size())
  {
    LineSegment2d lower_seg(point_pairs->at(i).first, point_pairs->at(j + 1).first);
    LineSegment2d upper_seg(point_pairs->at(i).second, point_pairs->at(j + 1).second);
    if (!IsPointNear(lower_seg, point_pairs->at(j).first, kMaxDist) ||
        !IsPointNear(upper_seg, point_pairs->at(j).second, kMaxDist))
    {
      ++i;
      if (i != j)
      {
        point_pairs->at(i) = point_pairs->at(j);
      }
    }
    ++j;
  }
  point_pairs->at(++i) = point_pairs->back();
  point_pairs->resize(i + 1);
}

//判断输入参数ST点在ST边界范围内？
bool ST_Boundary::IsPointInBoundary(const STPoint &st_point) const
{
  //输入的ST点的t在ST边界的范围内？t,s均没有超出最大最小范围？
  //若超出，说明该点就不在ST边界上
  if (st_point.t() <= min_t_ || st_point.t() >= max_t_)
  {
    return false;
  }
  //定义临时变量left,right size_t类似无符号整型
  size_t left = 0;
  size_t right = 0;
  //用给定的ST点的时间t去lower_points_去找位于哪两个点之间，这个区间[left,right]
  //找到的区间左下标放在left，右下标放在right
  if (!GetIndexRange(lower_points_, st_point.t(), &left, &right))
  {
    // std::cout << "failed to get index range.";
    return false;
  }
  //这一块不懂数学原理，但大致推测是计算几何的内容，
  //上述判断有没有超出t,和s的最大最小范围属于快速排斥实验
  //下面通过叉乘计算判断点和线段的位置叫跨立实验，暂不深究
  //CrossProd函数是Apollo自己写的向量叉乘的函数
  const double check_upper = common::math::CrossProd(
      st_point, upper_points_[left], upper_points_[right]);
  const double check_lower = common::math::CrossProd(
      st_point, lower_points_[left], lower_points_[right]);

  //若check_upper和check_lower 异号则说明给定ST点在ST边界范围内
  return (check_upper * check_lower < 0);
}

//查询当前时间对应的ST边界的s范围，输入参数是当前时间curr_time
//查询结果放入上边界指针s_upper，下边界指针s_lower中
bool ST_Boundary::GetBoundarySRange(const double curr_time, double *s_upper, double *s_lower) const
{
  // CHECK_NOTNULL(s_upper);
  // CHECK_NOTNULL(s_lower);
  //如果当前时间curr_time不在ST边界涵盖的时间范围内，直接返回false
  if (curr_time < min_t_ || curr_time > max_t_)
  {
    return false;
  }

  //当前时间所处的时间区间[left,right],left,right调用GetIndexRange去查询
  //查询curr_time所处的时间区间[left,right]，结果放入left,right
  size_t left = 0;
  size_t right = 0;
  if (!GetIndexRange(lower_points_, curr_time, &left, &right))
  {
    // AERROR << "Fail to get index range.";
    return false;
  }
  //r是ratio,若left=right ratio就是0
  //curr_time所处的时间区间[left,right]
  //否则ratio r = (当前时间-上边界[left这个时间对应点]的时间)/(上边界[right这个时间对应点]的时
  //间-上边界[left这个时间对应点]的时间),起始就是当前时间既然已经在left和right这个区间内，
  //那么 r=当前时间到left的时间距离/right-left时间的距离就是这个点到左边距离占整个区间长度的比例
  const double r =
      (left == right ? 0.0 : (curr_time - upper_points_[left].t()) / (upper_points_[right].t() - upper_points_[left].t()));

  //当前时间所处的时间区间的左时间left对应的上边界点的s +
  //比例r * (时间区间的右时间right对应的上边界点s - 时间区间的左时间left对应的上边界点s)
  //其实不就是根据当前时间curr_time去已知的ST上边界点插值吗
  *s_upper = upper_points_[left].s() +
             r * (upper_points_[right].s() - upper_points_[left].s());
  //就是根据当前时间curr_time去已知的ST下边界点插值
  *s_lower = lower_points_[left].s() +
             r * (lower_points_[right].s() - lower_points_[left].s());

  //FLAGS_speed_lon_decision_horizon是gflags老套路是去
  //modules\planning\common\planning_gflags.cc里取出speed_lon_decision_horizon的值
  //speed_lon_decision_horizon默认为200m，就是s边界的上界插值点和200m之间的较小值
  *s_upper = std::fmin(*s_upper, Config_.FLAGS_speed_lon_decision_horizon);
  //当前时间对应的s下边界至少>=0
  *s_lower = std::fmax(*s_lower, 0.0);
  return true;
}

bool ST_Boundary::GetIndexRange(const std::vector<STPoint> &points, const double t, size_t *left, size_t *right) const
{
  //   CHECK_NOTNULL(left);
  //   CHECK_NOTNULL(right);
  if (t < points.front().t() || t > points.back().t())
  {
    // AERROR << "t is out of range. t = " << t;
    return false;
  }
  auto comp = [](const STPoint &p, const double t)
  { return p.t() < t; };
  auto first_ge = std::lower_bound(points.begin(), points.end(), t, comp);
  size_t index = std::distance(points.begin(), first_ge);
  if (index == 0)
  {
    *left = *right = 0;
  }
  else if (first_ge == points.end())
  {
    *left = *right = points.size() - 1;
  }
  else
  {
    *left = index - 1;
    *right = index;
  }
  return true;
}

//获取未被阻塞的s范围
//输入参数是当前时间curr_time，当前时间对应的S上边界值，当前时间对应的S下边界值
//获取的未被阻塞的s的范围放在s_upper，s_lower里
//其实该函数的作用就是先用当前时间去当前ST边界上插值出此时的上下边界点，然后根据
//场景，需要加速的场景将ST下边界置为插值出的上边界点，压着限速走，需要减速停车的场景将ST上边界置
//为插值出的下边界点，压着低速走
bool ST_Boundary::GetUnblockSRange(const double curr_time, double *s_upper, double *s_lower) const
{
  // CHECK_NOTNULL(s_upper);
  // CHECK_NOTNULL(s_lower);

  //初始s上界为FLAGS_speed_lon_decision_horizon, s下界为0m
  *s_upper = Config_.FLAGS_speed_lon_decision_horizon;
  *s_lower = 0.0;
  //如果当前时间小于ST边界最小值，或超出最大时间直接返回true，那意思就是整个
  //0-200m都未被阻塞了
  // std::cout << min_t_ << "," << max_t_ << "\n";
  if (curr_time < min_t_ || curr_time > max_t_)
  {
    return true;
  }

  //找出当前时间在ST边界上对应的时间区间[left,right]
  size_t left = 0;
  size_t right = 0;
  if (!GetIndexRange(lower_points_, curr_time, &left, &right))
  {
    // std::cout << "Fail to get index range.";
    return false;
  }
  // std::cout << "left:" << left << ","
  //           << "right:" << right << "\n";
  //如果当前时间大于上边界点的right时间对应点的时间，直接返回true
  if (curr_time > upper_points_[right].t())
  {
    return true;
  }
  //用定义比例r是为了根据当前时间去插值出s?
  const double r =
      (left == right ? 0.0 : (curr_time - upper_points_[left].t()) / (upper_points_[right].t() - upper_points_[left].t()));

  //当前时间对应的上边界点s
  double upper_cross_s = upper_points_[left].s() + r * (upper_points_[right].s() - upper_points_[left].s());

  //当前时间对应的下边界点s
  double lower_cross_s = lower_points_[left].s() + r * (lower_points_[right].s() - lower_points_[left].s());

  // std::cout << "upper_points_." << upper_points_[0].s() << "," << upper_points_[0].t() << "\n";
  // std::cout << "upper_points_." << upper_points_[1].s() << "," << upper_points_[1].t() << "\n";
  // std::cout << "lower_points_." << lower_points_[0].s() << "," << lower_points_[0].t() << "\n";
  // std::cout << "lower_points_." << lower_points_[1].s() << "," << lower_points_[1].t() << "\n";

  //如果ST边界的类型是STOP停止或者YIELD减速避让或者FOLLOW跟车
  if (boundary_type_ == BoundaryType::STOP || boundary_type_ == BoundaryType::YIELD ||
      boundary_type_ == BoundaryType::FOLLOW)
  {
    *s_upper = lower_cross_s;
  }
  //ST边界类型是OVERTAKE超车
  //则将下边界点置为当前时间插值出来的上边界s，和原有下边界的较大值
  //其实就是压着当前时间插值出的上边界S走？
  else if (boundary_type_ == BoundaryType::OVERTAKE)
  {
    *s_lower = std::fmax(*s_lower, upper_cross_s);
  }
  else
  {
    // std::cout << "boundary_type is not supported. boundary_type: " << static_cast<int>(boundary_type_);
    return false;
  }
  return true;
}

//该函数的作用是获取边界线的斜率？从命名来看
//输入参数是curr_time当前时间，ds_upper，ds_lower推测应该是计算的
//上下边界的变化率计算出后放入ds_upper，ds_lower是函数的计算结果
bool ST_Boundary::GetBoundarySlopes(const double curr_time, double *ds_upper,
                                    double *ds_lower) const
{
  //若有空指针则直接返回false
  if (ds_upper == nullptr || ds_lower == nullptr)
  {
    return false;
  }
  //如果当前时间不在STBoundary所覆盖的ST点时间范围内，也直接返回false
  if (curr_time < min_t_ || curr_time > max_t_)
  {
    return false;
  }

  //静态常量kTimeIncrement 明显是一个时间增量为0.05
  static constexpr double kTimeIncrement = 0.05;
  //用当前时间curr_time - 一个时间增量得到前继点的t
  double t_prev = curr_time - kTimeIncrement;
  //初始化前继点的s上下边界为0
  double prev_s_upper = 0.0;
  double prev_s_lower = 0.0;
  //调用GetBoundarySRange函数就根据时间t去STBoundary中查询出相应时间对应的s的上下边界
  //查询结果放在prev_s_upper前继点s的上边界，prev_s_lower前继点s的下边界
  bool has_prev = GetBoundarySRange(t_prev, &prev_s_upper, &prev_s_lower);
  //定义后继点的时间为当前时间再加上时间增量kTimeIncrement
  double t_next = curr_time + kTimeIncrement;
  //初始化后继点的s上下边界为0
  double next_s_upper = 0.0;
  double next_s_lower = 0.0;
  //调用GetBoundarySRange函数就根据时间t去STBoundary中查询出相应时间对应的s的上下边界
  //查询结果放在next_s_upper后继点s的上边界，next_s_lower后继点s的下边界
  bool has_next = GetBoundarySRange(t_next, &next_s_upper, &next_s_lower);
  //同上述前继点，后继点的操作求出当前时间对应的上下s边界 curr_s_upper  curr_s_lower
  double curr_s_upper = 0.0;
  double curr_s_lower = 0.0;
  GetBoundarySRange(curr_time, &curr_s_upper, &curr_s_lower);
  //如果既没有前继点又没有后继点条件直接返回false
  if (!has_prev && !has_next)
  {
    return false;
  }
  //如果既有前继点又有后继点，则利用三点信息计算上下边界的纵向位置s变化率，并直接返回true
  if (has_prev && has_next)
  {
    *ds_upper = ((next_s_upper - curr_s_upper) / kTimeIncrement +
                 (curr_s_upper - prev_s_upper) / kTimeIncrement) *
                0.5;
    *ds_lower = ((next_s_lower - curr_s_lower) / kTimeIncrement +
                 (curr_s_lower - prev_s_lower) / kTimeIncrement) *
                0.5;
    return true;
  }
  //如果只有前继点没有后继点，则利用当前和前继点信息计算上下边界的纵向位置s变化率
  if (has_prev)
  {
    *ds_upper = (curr_s_upper - prev_s_upper) / kTimeIncrement;
    *ds_lower = (curr_s_lower - prev_s_lower) / kTimeIncrement;
  }
  else
  {
    //如果只有后继点没有前继点，则利用当前和后继点信息计算上下边界的纵向位置s变化率
    *ds_upper = (next_s_upper - curr_s_upper) / kTimeIncrement;
    *ds_lower = (next_s_lower - curr_s_lower) / kTimeIncrement;
  }
  return true;
}
