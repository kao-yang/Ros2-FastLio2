#pragma once

namespace fast_lio::options {

/// fixed params
constexpr double MOV_THRESHOLD = 1.5;       // 雷达边界的扩张系数，与localmap的边界有关
constexpr double INIT_TIME = 0.1;
constexpr double LASER_POINT_COV = 0.001;
constexpr int PUBFRAME_PERIOD = 20;
constexpr int NUM_MATCH_POINTS = 5;      // required matched points in current
constexpr int MIN_NUM_MATCH_POINTS = 3;  // minimum matched points in current

/// configurable params
extern int NUM_MAX_ITERATIONS;      // max iterations of ekf
extern float ESTI_PLANE_THRESHOLD;  // plane threshold
extern bool FLAG_EXIT;              // flag for exitting

}  // namespace fast_lio::options

