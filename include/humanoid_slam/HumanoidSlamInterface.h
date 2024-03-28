#pragma once

#include <string>
#include <tuple>

#include "sensor/ImuData.h"
#include "sensor/TimedPointCloudData.h"
#include "humanoid_slam/sensor/PoseData.h"
#include "sophus/se3.hpp"

namespace humanoid_slam{

struct SensorId {
    enum class SensorType {
    RANGE = 0,
    IMU
    };

    SensorType type;
    std::string sId;

    bool operator==(const SensorId& other) const {
    return std::forward_as_tuple(type, sId) ==
            std::forward_as_tuple(other.type, other.sId);
    }

    bool operator<(const SensorId& other) const {
    return std::forward_as_tuple(type, sId) <
            std::forward_as_tuple(other.type, other.sId);
    }
};

class HumanoidSlamInterface{
public:
    HumanoidSlamInterface() {}
    virtual ~HumanoidSlamInterface() {}

    HumanoidSlamInterface(const HumanoidSlamInterface&) = delete;
    HumanoidSlamInterface& operator=(const HumanoidSlamInterface&) = delete;

    virtual void AddSensorData( const std::string& sSensorId, 
                const sensor::TimedPointCloudData& cloudData ) = 0;
    virtual void AddSensorData( const std::string& sSensorId, 
                const sensor::ImuData& imuData ) = 0;
    virtual std::tuple< const sensor::TimedPointCloudData, const sensor::TimedPointCloudData,
                        const sensor::PoseData > GetLidarOdomData() = 0;

private:
};

} // namespace humanoid_slam