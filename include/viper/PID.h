#pragma once

#include <cmath>
// #include <algorithm>
#include <chrono>

#include "lpf.h"

namespace viper {

    class PID {
    public:

        float p, i, d;
        float windup_limit;
        LowPassFilter<float> lpf;
        float dtMax;

        float derivative = 0;
        float integral = 0;

        PID(float p, float i, float d, float windup = 0, float dAlpha = 1, float dtMax = 0.1)
            : p(p), i(i), d(d), windup_limit(windup), lpf(dAlpha), dtMax(dtMax) {}

        float update(float error) {
            using clock = std::chrono::steady_clock;

            auto now = clock::now();

            float dt = 0.0f;

            if (initialized) {
                dt = std::chrono::duration<float>(now - prevTime).count();
            }

            if (initialized && dt > 0 && dt < dtMax) {
                integral += error * dt;
                derivative = lpf.update((error - prevError) / dt);
            } else {
                integral = 0;
                derivative = 0;
            }

            prevError = error;
            prevTime = now;
            initialized = true;
            
            // NOTE: Flix clamps output directly: integral = std::clamp(i * integral, -windup, windup)
            // Clamp integral to prevent windup (though if i too small, integral term may be very small)
            if (windup_limit > 0.0f) {
                integral = std::clamp(integral, -windup_limit, windup_limit);
            }

            return p * error
                + i * integral
                + d * derivative;
        }

        void reset() {
            initialized = false;
            integral = 0;
            derivative = 0;
            lpf.reset();
        }

    private:
        using clock = std::chrono::steady_clock;

        clock::time_point prevTime;
        float prevError = 0;
        bool initialized = false;
    };

}  // namespace viper
