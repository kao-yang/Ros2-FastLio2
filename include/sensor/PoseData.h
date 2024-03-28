#pragma once
#include <Eigen/Core>
#include "sophus/se3.hpp"

namespace humanoid_slam{
namespace sensor{

struct PoseData{
    double fTime = -1;
    std::string sFrameId;
    std::string sChildFrameId;
    Sophus::SE3d pose;
    Eigen::Vector3d bodyAngularVelocity;// = Eigen::Vector3d(3,0);
    Eigen::Vector3d bodyLinearVelocity;// =  Eigen::Vector3d(3,0);
    
};

} // namespace sensor
} // namespace humanoid_slam