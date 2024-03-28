#pragma once
#include <deque>
#include "ImuData.h"
#include "TimedPointCloudData.h"


namespace humanoid_slam{
namespace sensor{

struct LidarOdomMeasureData {
    std::shared_ptr< sensor::TimedPointCloudData > pLidar;
    std::deque< std::shared_ptr<sensor::ImuData> > qPImu;
};

} // namespace sensor
} // namespace humanoid_slamG