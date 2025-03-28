#ifndef TRAJECTORY_STRUCT_HPP
#define TRAJECTORY_STRUCT_HPP

#include <cstdint>

struct TrajectoryStruct {
  double x;
  double y;
  double z;
  int32_t sec;
  uint32_t nsec;
};

#endif