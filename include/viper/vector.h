#pragma once

#include <cmath>
#include <iostream>
#include <algorithm>

#include "util.h"

namespace viper {

    class Vector {
    public:
        float x, y, z;

        Vector() : x(0), y(0), z(0) {}
        Vector(float x, float y, float z) : x(x), y(y), z(z) {}

        bool zero() const {
            const float eps = 1e-6f;
            return std::abs(x) < eps && std::abs(y) < eps && std::abs(z) < eps;
        }

        bool finite() const {
            return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
        }

        bool valid() const {
            return finite();
        }

        bool invalid() const {
            return !valid();
        }

        void invalidate() {
            x = NAN;
            y = NAN;
            z = NAN;
        }

        float norm() const {
            return std::sqrt(x * x + y * y + z * z);
        }

        void normalize() {
            float n = norm();
            if (n > 1e-6f) {
                x /= n;
                y /= n;
                z /= n;
            }
        }

        // Zero out vector
        void reset() {
            x = y = z = 0;
        }

        Vector operator+(float b) const {
            return Vector(x + b, y + b, z + b);
        }

        Vector operator*(float b) const {
            return Vector(x * b, y * b, z * b);
        }

        Vector operator/(float b) const {
            return Vector(x / b, y / b, z / b);
        }

        Vector operator+(const Vector& b) const {
            return Vector(x + b.x, y + b.y, z + b.z);
        }

        Vector operator-(const Vector& b) const {
            return Vector(x - b.x, y - b.y, z - b.z);
        }

        Vector& operator+=(const Vector& b) {
            return *this = *this + b;
        }

        Vector& operator-=(const Vector& b) {
            return *this = *this - b;
        }

        // Element-wise multiplication
        Vector operator*(const Vector& b) const {
            return Vector(x * b.x, y * b.y, z * b.z);
        }

        // Element-wise division
        Vector operator/(const Vector& b) const {
            return Vector(x / b.x, y / b.y, z / b.z);
        }

        bool operator==(const Vector& b) const {
            const float eps = 1e-6f;
            return std::abs(x - b.x) < eps &&
                std::abs(y - b.y) < eps &&
                std::abs(z - b.z) < eps;
        }

        bool operator!=(const Vector& b) const {
            return !(*this == b);
        }

        static float dot(const Vector& a, const Vector& b) {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        static Vector cross(const Vector& a, const Vector& b) {
            return Vector(
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x
            );
        }

        static float angleBetween(const Vector& a, const Vector& b) {
            float denom = a.norm() * b.norm();
            if (denom < 1e-6f) return 0.0f;

            float cos_theta = dot(a, b) / denom;
            cos_theta = std::clamp(cos_theta, -1.0f, 1.0f);

            return std::acos(cos_theta);
        }

        static Vector rotationVectorBetween(const Vector& a, const Vector& b) {
            float an = a.norm();
            float bn = b.norm();
            if (an < 1e-6 || bn < 1e-6) {
                return Vector(0, 0, 0);
            }
            Vector direction = cross(a, b);
            if (direction.norm() < 1e-6) { // vectors are parallel
                if (dot(a, b) > 0) { // same direction
                    return Vector(0, 0, 0);
                }
                // opposite direction
                Vector perp = cross(a, Vector(1, 0, 0));
                if (perp.norm() < 1e-6) {
                    perp = cross(a, Vector(0, 1, 0));
                }
                perp.normalize();
                return perp * PI;
            }
            direction.normalize();
            float angle = angleBetween(a, b);
            return direction * angle;
        }
    };

    inline Vector operator*(float a, const Vector& b) { return b * a; }
    inline Vector operator+(float a, const Vector& b) { return b + a; }


}