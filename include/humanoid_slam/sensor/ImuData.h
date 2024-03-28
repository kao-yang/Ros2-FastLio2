#pragma once
#include <Eigen/Core>


namespace humanoid_slam{
namespace sensor{

struct ImuData{
    double fTime = -1;
    Eigen::Vector3d angularVelocity;// = Eigen::Vector3d(3,0);
    Eigen::Vector3d linearAcceleration;// =  Eigen::Vector3d(3,0);
};

} // namespace sensor
} // namespace humanoid_slam