/**
 * Defines the AttitudeController and RateController classes. Could be combined into a single one later.
 */
#pragma once

#include <algorithm>
#include "PID.h"
#include "vector.h"
#include "quaternion.h"

// !TODO: Move all the default values below into #defines here

// Maximum angular rates (roll and pitch rates managed by attitude controller)
#define YAWRATE_MAX   radians(30)

namespace viper {

/**
 * @brief Attitude controller that produces target angular rates from attitude error
 * 
 * This is the outer loop in a cascaded control system:
 * - Input: current attitude (quaternion), target attitude (quaternion)
 * - Process: compute attitude error → apply PID gains → output target rates
 * - Output: target angular rates (rad/s)
 * 
 * The controller uses 3 separate PID loops:
 * - Roll PID: controls roll angle error → target roll rate
 * - Pitch PID: controls pitch angle error → target pitch rate
 * - Yaw PID: controls yaw angle error → target yaw rate
 */
class AttitudeController {
public:
    /**
     * @brief Constructor with default PID gains from flix reference. I, D default to 0
     * 
     * @param roll_p    Roll proportional gain (default: 6.0)
     * @param pitch_p   Pitch proportional gain (default: 6.0)
     * @param yaw_p     Yaw proportional gain (default: 3.0)
     */
    AttitudeController(float roll_p = 0.2f, float roll_i = 0.3f, float roll_d = 0.05f,
                       float pitch_p = 0.2f, float pitch_i = 0.3f, float pitch_d = 0.05f,
                       float yaw_p = 3.0f, float yaw_i = 0.0f, float yaw_d = 0.0f,
                       float roll_damping = 1.0f, float pitch_damping = 1.0f)
        : roll_pid_(roll_p, roll_i, roll_d),
          pitch_pid_(pitch_p, pitch_i, pitch_d),
          yaw_pid_(yaw_p, yaw_i, yaw_d),
          roll_damping_(std::clamp(roll_damping, 0.0f, 1.0f)),
          pitch_damping_(std::clamp(pitch_damping, 0.0f, 1.0f)) {}

    /**
     * @brief Update attitude controller and compute target rates
     * 
     * @param attitude_current Current quaternion attitude (w, x, y, z)
     * @param attitude_target  Target quaternion attitude (w, x, y, z)
     * @return Vector3 Target angular rates (x=roll_rate, y=pitch_rate, z=yaw_rate) in rad/s
     */
    Vector update(const Quaternion& attitude_current, const Quaternion& attitude_target, const Vector& rates_extra = Vector(0, 0, 0)) {
        // Compute attitude error by comparing Vector form of the two arguments
        const Vector up(0, 0, 1);
        Vector upActual = Quaternion::rotateVector(up, attitude_current);
        Vector upTarget = Quaternion::rotateVector(up, attitude_target);
        // Get the error between them
        Vector error = Vector::rotationVectorBetween(upTarget, upActual);

        // Apply PID gains to get target rates
        float roll_rate_target = roll_pid_.update(error.x);
        float pitch_rate_target = pitch_pid_.update(error.y);
        // Yaw must be computed from the relative rotation about the up axis
        // (rotation to bring current heading to target heading). Use
        // quaternion between to get the relative yaw error and add
        // feed-forward `rates_extra.z` (e.g. commanded yaw rate).
        Quaternion q_err = Quaternion::between(attitude_target, attitude_current);
        float yaw_error = wrapAngle(q_err.getYaw());
        float yaw_rate_target = yaw_pid_.update(yaw_error) + rates_extra.z;

        return Vector(roll_rate_target, pitch_rate_target, yaw_rate_target);
    }

    /**
     * @brief Set all attitude controller gains
     */
    void set_gains(float roll_p, float pitch_p, float yaw_p) {
        roll_pid_.p = roll_p;
        pitch_pid_.p = pitch_p;
        yaw_pid_.p = yaw_p;
    }

    void set_gains(float roll_p, float roll_i, float roll_d,
                   float pitch_p, float pitch_i, float pitch_d,
                   float yaw_p, float yaw_i, float yaw_d,
                   float roll_damping = 1.0f, float pitch_damping = 1.0f) {
        roll_pid_.p = roll_p;
        roll_pid_.i = roll_i;
        roll_pid_.d = roll_d;

        pitch_pid_.p = pitch_p;
        pitch_pid_.i = pitch_i;
        pitch_pid_.d = pitch_d;

        yaw_pid_.p = yaw_p;
        yaw_pid_.i = yaw_i;
        yaw_pid_.d = yaw_d;

        roll_damping_ = std::clamp(roll_damping, 0.0f, 1.0f);
        pitch_damping_ = std::clamp(pitch_damping, 0.0f, 1.0f);
    }

    /**
     * @brief Get roll PID controller for parameter adjustment
     */
    PID& get_roll_pid() { return roll_pid_; }

    /**
     * @brief Get pitch PID controller for parameter adjustment
     */
    PID& get_pitch_pid() { return pitch_pid_; }

    /**
     * @brief Get yaw PID controller for parameter adjustment
     */
    PID& get_yaw_pid() { return yaw_pid_; }

    /**
     * @brief Reset all PID controller states
     */
    void reset() {
        roll_pid_.reset();
        pitch_pid_.reset();
        yaw_pid_.reset();
    }

private:
    PID roll_pid_;   ///< PID controller for roll angle
    PID pitch_pid_;  ///< PID controller for pitch angle
    PID yaw_pid_;    ///< PID controller for yaw angle
    float roll_damping_ = 1.0f;  ///< Outer attitude damping for roll angle control
    float pitch_damping_ = 1.0f; ///< Outer attitude damping for pitch angle control
};



/**
 * @brief Angular rate controller that produces motor torques from rate error
 * 
 * This is the inner loop in a cascaded control system:
 * - Input: target angular rates (rad/s), current angular rates (rad/s from gyro)
 * - Process: compute rate error → apply PID gains → output torque commands
 * - Output: target torques (normalized to [-1, 1] range for motor mixing)
 * 
 * The controller uses 3 separate PID loops (one per axis):
 * - Roll rate PID: controls roll rate error → roll torque
 * - Pitch rate PID: controls pitch rate error → pitch torque
 * - Yaw rate PID: controls yaw rate error → yaw torque
 */
class RateController {
public:
    /**
     * @brief Constructor with default flix rate control gains
     * 
     * Rate controllers typically use higher gains to respond quickly to disturbances.
     * 
     * @param roll_p       Roll rate proportional gain (default: 0.15)
     * @param roll_i       Roll rate integral gain (default: 0.2)
     * @param roll_d       Roll rate derivative gain (default: 0.0002)
     * @param pitch_p      Pitch rate proportional gain (default: 0.15)
     * @param pitch_i      Pitch rate integral gain (default: 0.2)
     * @param pitch_d      Pitch rate derivative gain (default: 0.0002)
     * @param yaw_p        Yaw rate proportional gain (default: 0.3)
     * @param yaw_i        Yaw rate integral gain (default: 0.05)
     * @param yaw_d        Yaw rate derivative gain (default: 0.00015)
     * @param windup       Integral windup limit (default: 0.3)
     */
    RateController(float roll_p = 0.15f, float roll_i = 0.2f, float roll_d = 0.0002f,
                   float pitch_p = 0.15f, float pitch_i = 0.2f, float pitch_d = 0.0002f,
                   float yaw_p = 0.3f, float yaw_i = 0.05f, float yaw_d = 0.00015f,
                   float windup = 0.3f, float d_lpf_alpha = 0.2f)
        : roll_pid_(roll_p, roll_i, roll_d, windup, d_lpf_alpha),
          pitch_pid_(pitch_p, pitch_i, pitch_d, windup, d_lpf_alpha),
          yaw_pid_(yaw_p, yaw_i, yaw_d, windup, d_lpf_alpha) {}

    /**
     * @brief Update rate controller and compute target torques
     * 
     * @param rate_target   Target angular rates (rad/s) - from attitude controller
     * @param rate_current  Current angular rates (rad/s) - from gyroscope
     * @return Vector3 Target torques (x=roll_torque, y=pitch_torque, z=yaw_torque) in [-1, 1]
     */
    Vector update(const Vector& rate_target, const Vector& rate_current) {
        // Compute rate errors
        Vector error = rate_target - rate_current;

        // Apply PID gains to get target torques
        float roll_torque = roll_pid_.update(error.x);
        float pitch_torque = pitch_pid_.update(error.y);
        float yaw_torque = yaw_pid_.update(error.z);

        return Vector(roll_torque, pitch_torque, yaw_torque);
    }

    /**
     * @brief Set all rate controller gains
     */
    void set_gains(float roll_p, float roll_i, float roll_d,
                   float pitch_p, float pitch_i, float pitch_d,
                   float yaw_p, float yaw_i, float yaw_d) {
        roll_pid_.p = roll_p;
        roll_pid_.i = roll_i;
        roll_pid_.d = roll_d;

        pitch_pid_.p = pitch_p;
        pitch_pid_.i = pitch_i;
        pitch_pid_.d = pitch_d;

        yaw_pid_.p = yaw_p;
        yaw_pid_.i = yaw_i;
        yaw_pid_.d = yaw_d;
    }

    /**
     * @brief Get roll rate PID controller for parameter adjustment
     */
    PID& get_roll_pid() { return roll_pid_; }

    /**
     * @brief Get pitch rate PID controller for parameter adjustment
     */
    PID& get_pitch_pid() { return pitch_pid_; }

    /**
     * @brief Get yaw rate PID controller for parameter adjustment
     */
    PID& get_yaw_pid() { return yaw_pid_; }

    /**
     * @brief Reset all PID controller states
     */
    void reset() {
        roll_pid_.reset();
        pitch_pid_.reset();
        yaw_pid_.reset();
    }

private:
    PID roll_pid_;   ///< PID controller for roll rate
    PID pitch_pid_;  ///< PID controller for pitch rate
    PID yaw_pid_;    ///< PID controller for yaw rate
};

}  // namespace viper