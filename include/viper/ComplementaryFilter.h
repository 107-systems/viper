/**
 * ComplementaryFilter.h
 *
 * Fuses raw accel + gyro. Ignores msg.orientation entirely.
 * Uses quaternion-domain gyro integration to avoid gimbal lock.
 */

#pragma once

#include "EstimatorBase.h"
#include "lpf.h"

namespace viper
{

class ComplementaryFilter final : public EstimatorBase
{
public:
  ComplementaryFilter()
  : EstimatorBase("complementary")
  {}

  float acc_weight{0.003f};
  LowPassFilter<Vector> rates_filter{0.2f};

protected:
  AttitudeEstimate on_update(sensor_msgs::msg::Imu const & msg, float dt) override
  {
    Vector gyro(
      static_cast<float>(msg.angular_velocity.x),
      static_cast<float>(msg.angular_velocity.y),
      static_cast<float>(msg.angular_velocity.z));

    Vector acc(
      static_cast<float>(msg.linear_acceleration.x),
      static_cast<float>(msg.linear_acceleration.y),
      static_cast<float>(msg.linear_acceleration.z));

    if (!_initialised)
    {
      // Seed attitude from gravity, assuming IMU is stationary
      Vector up = acc * (1.0f / acc.norm());
      _attitude = Quaternion::fromBetweenVectors(up, Vector(0, 0, 1));
      _attitude.normalize();
      rates_filter.output = gyro;
      _initialised = true;
    }
    
    // Raw gyro for dead-reckoning
    if (dt > 0.0f)
    _attitude = Quaternion::rotate(_attitude, Quaternion::fromRotationVector(gyro * dt));
    
    // Filtered rates: D-term consumption only
    _rates = rates_filter.update(gyro);

    // Accelerometer gravity correction
    apply_acc(acc);

    AttitudeEstimate est;
    est.orientation = _attitude;
    est.angular_rate = _rates;
    est.valid = _attitude.valid();
    return est;
  }

  void on_reset() override
  {
    _attitude = Quaternion();
    _rates = Vector();
    _initialised = false;
    rates_filter.reset();
  }

private:
  void apply_acc(Vector const & acc)
  {
    Vector up = Quaternion::rotateVector(Vector(0, 0, 1), _attitude);
    // Slowly adjust accelerometer back to true 'up'
    Vector correction = Vector::rotationVectorBetween(up, acc) * acc_weight;
    _attitude = Quaternion::rotate(_attitude, Quaternion::fromRotationVector(correction));
  }

  Quaternion _attitude;
  Vector     _rates;
  bool       _initialised{false};
};

} /* viper */