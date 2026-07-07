#pragma once

#include <algorithm>
#include "vector.h"

namespace viper {

/**
 * @brief Motor mixer for X-quadrotor configuration
 * 
 * Converts collective thrust and torque commands into individual motor outputs.
 * 
 * Motor Layout (X-configuration):
 * 
 *     3(FL) --------- 2(FR)
 *        | \       / |
 *        |   \   /   |
 *        |     X     |
 *        |   /   \   |
 *        | /       \ |
 *     0(RL) --------- 1(RR)
 * 
 * Motor rotation directions:
 * - 0 (Rear-Left):   CCW
 * - 1 (Rear-Right):  CW
 * - 2 (Front-Right): CCW
 * - 3 (Front-Left):  CW
 * 
 * Motor mixing formulas (thrust + torques → motor outputs):
 * 
 *     motor[0] = thrust + τ_roll - τ_pitch + τ_yaw  (RL)
 *     motor[1] = thrust - τ_roll - τ_pitch - τ_yaw  (RR)
 *     motor[2] = thrust + τ_roll + τ_pitch - τ_yaw  (FR)
 *     motor[3] = thrust - τ_roll + τ_pitch + τ_yaw  (FL)
 * 
 * Where:
 * - thrust:   collective motor command [0, 1]
 * - τ_roll:   roll torque in [-1, 1]
 * - τ_pitch:  pitch torque in [-1, 1]
 * - τ_yaw:    yaw torque in [-1, 1]
 */
class MotorMixer {
public:
    /**
     * @brief Mix thrust and torques into 4 motor commands
     * 
     * @param thrust Target collective thrust [0.0, 1.0], where 0=idle, 1=max
     * @param torques Target torques vector (x=roll, y=pitch, z=yaw) in normalized [-1, 1]
     * @return Array of 4 motor commands [motor0, motor1, motor2, motor3] in [0.0, 1.0]
     * 
     * @note Output motor values are clamped to [0.0, 1.0] range
     * @note Safety: if thrust < 0.05, all motors are set to 0 (disarmed)
     */
    std::array<float, 4> mix(float thrust, const Vector& torques) {
        std::array<float, 4> motors;

        // Safety: disarm if thrust is too low
        if (thrust < 0.05f) {
            motors.fill(0.0f);
            return motors;
        }

        // X-quad motor mixing formulas
        // These determine how thrust and torques map to individual motors
        // motors[3] = thrust - torques.x + torques.y + torques.z;  // FL
        // motors[2] = thrust + torques.x + torques.y - torques.z;  // FR
        // motors[0] = thrust + torques.x - torques.y + torques.z;  // RL
        // motors[1] = thrust - torques.x - torques.y - torques.z;  // RR

        // Roll: Left (+) Right (-) | Pitch: Front (+) Rear (-) | Yaw: CW (+) CCW (-)
        motors[3] = thrust + torques.x - torques.y + torques.z;  // FL (Left, Front, CW)
        motors[2] = thrust - torques.x - torques.y - torques.z;  // FR (Right, Front, CCW)
        motors[0] = thrust + torques.x + torques.y - torques.z;  // RL (Left, Rear, CCW)
        motors[1] = thrust - torques.x + torques.y + torques.z;  // RR (Right, Rear, CW)

        // Clamp motors to valid range [0.0, 1.0]
        for (auto& motor : motors) {
            motor = std::clamp(motor, 0.0f, 1.0f);
        }

        return motors;
    }

    /**
     * @brief Get motor output limits for this quadrotor
     * 
     * @return Maximum motor torque magnitude that can be applied
     * @note Limits based on physical motor constraints
     */
    float get_max_torque() const {
        // For symmetric quad: max torque is limited by max motor speed
        // Assuming motors 0,1,2,3 can each provide ±1 thrust
        // Maximum achievable torque ≈ 1/2 (from opposing motor pairs)
        return 0.5f;
    }

    /**
     * @brief Scale torque command to respect motor limits
     * 
     * If the motor mixing would saturate any motor,
     * this function scales down the torque to fit within [0, 1] limits.
     * 
     * @param thrust Collective thrust [0.0, 1.0]
     * @param torques Target torques vector
     * @return Scaled torques that respect motor output limits
     */
    Vector limit_torques(float thrust, const Vector& torques) {
        // Compute mixing extremes
        std::array<float, 4> motors = mix(thrust, torques);

        // Find maximum saturation
        float max_saturation = 1.0f;

        for (const auto& motor : motors) {
            if (motor < 0.0f) {
                max_saturation = std::min(max_saturation, -motor);
            } else if (motor > 1.0f) {
                max_saturation = std::min(max_saturation, 1.0f / motor);
            }
        }

        // Scale down torques to prevent saturation
        return torques * max_saturation;
    }

private:
    // Motor configuration constants (can be extended for other quad types)
    static constexpr int NUM_MOTORS = 4;
};

}  // namespace viper