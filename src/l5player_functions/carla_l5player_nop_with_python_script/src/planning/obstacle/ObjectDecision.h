#pragma once
#include <string>

// dodge the obstacle in lateral direction when driving
class ObjectNudge
{
public:
  enum class Type
  {
    LEFT_NUDGE = 1,  //  障碍物标签为向左微调ObjectNudge::LEFT_NUDGE，并且无人车确实被障碍物阻挡
    RIGHT_NUDGE = 2, // drive from the right side of the obstacle
    NO_NUDGE = 3,    // No nudge is set.
  };
  Type type = Type::NO_NUDGE; //定死

  std::string TypeName() const
  {
    if (type == Type::LEFT_NUDGE)
    {
      return "LEFT_NUDGE";
    }
    else if (type == Type::RIGHT_NUDGE)
    {
      return "RIGHT_NUDGE";
    }
    else if (type == Type::NO_NUDGE)
    {
      return "NO_NUDGE";
    }
    return "";
  }

  // minimum lateral distance in meters. positive if type = LEFT_NUDGE
  // negative if type = RIGHT_NUDGE
  double distance_l = 2;
};

class ObjectSidePass
{
public:
  enum class Type
  {
    LEFT = 1,
    RIGHT = 2,
  };
  Type type = Type::LEFT;
};

class ObjectYield
{
public:
  double distance_s = 1; // minimum longitudinal distance in meters
  double fence_point = 2;
  double fence_heading = 3;
  double time_buffer = 4; // minimum time buffer required after the obstacle reaches the intersect point.
};

class ObjectFollow
{
public:
  double distance_s = 1; // minimum longitudinal distance in meters
  double fence_point = 2;
  double fence_heading = 3;
};

class ObjectOvertake
{
public:
  double distance_s = 1; // minimum longitudinal distance in meters
  double fence_point = 2;
  double fence_heading = 3;
  double time_buffer = 4; // minimum time buffer required before the obstacle reaches the intersect point.
};

//-----------------------------每个障碍物标签的类，先列机种要用到的--------------------------//
class ObjectDecisionType
{
public:
  enum class Type
  {
    OBJECT_TAG_NOT_SET = 1,
    OBJECT_TAG_HAS_SET = 2,
  };

  bool has_nudge() const
  {
    return nudge_.type != ObjectNudge::Type::NO_NUDGE;
  }
  ObjectNudge nudge() const
  {
    return nudge_;
  }

  Type object_tag_case() const
  {
    return type;
  }

  std::string TypeName() const
  {
    if (type == Type::OBJECT_TAG_NOT_SET)
    {
      return "OBJECT_TAG_NOT_SET";
    }
    else if (type == Type::OBJECT_TAG_HAS_SET)
    {
      return "OBJECT_TAG_HAS_SET";
    }
    return "";
  }

public:
  ObjectNudge nudge_;
  Type type = Type::OBJECT_TAG_HAS_SET;

  // bjectIgnore ignore = 1;
  // ObjectStop stop = 2;
  // ObjectFollow follow = 3;
  // ObjectYield yield = 4;
  // ObjectOvertake overtake = 5;
  // ObjectNudge nudge = 6;
  // ObjectAvoid avoid = 7;
};