#pragma once

#include "humanoid_slam/sensor/ImuData.h"
#include "humanoid_slam/sensor/TimedPointCloudData.h"
#include "humanoid_slam/sensor/PoseData.h"
#include "sophus/se3.hpp"

namespace humanoid_slam{
namespace lidar_odom{

class LidarOdomInterface{
public:

    LidarOdomInterface(){}
    virtual ~LidarOdomInterface(){}
    virtual bool RunOdom() = 0;
    virtual void InputData( const ::humanoid_slam::sensor::ImuData& imuData) = 0;
    virtual void InputData( const ::humanoid_slam::sensor::TimedPointCloudData& cloudData) = 0;
    virtual ::humanoid_slam::sensor::TimedPointCloudData GetUndistorCloud() = 0;
    virtual ::humanoid_slam::sensor::TimedPointCloudData GetLocalMap() = 0;
    virtual ::humanoid_slam::sensor::PoseData GetLocalPose() = 0;
};

} // namespace lidar_odom
} // namespace humanoid_slam