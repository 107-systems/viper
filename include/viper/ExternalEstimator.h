/**
 * ExternalEstimator.h
 *
 * Uses the orientation from BNO085 internal algorithm.
 */

#pragma once

#include "EstimatorBase.h"

namespace viper
{

class ExternalEstimator final : public EstimatorBase
{
public:
  ExternalEstimator()
  : EstimatorBase("ExternalEstimator")
  {}


protected:
  AttitudeEstimate on_update(sensor_msgs::msg::Imu const & msg, [[maybe_unused]] float dt) override
  {

    Vector gyro(
      static_cast<float>(msg.angular_velocity.x),
      static_cast<float>(msg.angular_velocity.y),
      static_cast<float>(msg.angular_velocity.z));

    Quaternion orientation(
      static_cast<float>(msg.orientation.w),
      static_cast<float>(msg.orientation.x),
      static_cast<float>(msg.orientation.y),
      static_cast<float>(msg.orientation.z));

    AttitudeEstimate est;
    est.orientation = orientation;
    est.angular_rate = gyro;
    est.valid = true;
    return est;
  }

  void on_reset() override
  {
    return;
  }

};

} /* viper */