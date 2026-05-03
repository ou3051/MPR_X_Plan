#pragma once

#include <array>

namespace measurement {

struct Vec3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Size3i {
    int x = 0;
    int y = 0;
    int z = 0;
};

struct Mat4d {
    std::array<double, 16> values{};

    static Mat4d identity();

    [[nodiscard]] double at(int row, int column) const;
    double& at(int row, int column);
};

[[nodiscard]] Vec3d operator+(Vec3d lhs, Vec3d rhs);
[[nodiscard]] Vec3d operator-(Vec3d lhs, Vec3d rhs);
[[nodiscard]] Vec3d operator*(Vec3d vector, double scalar);
[[nodiscard]] Vec3d operator*(double scalar, Vec3d vector);
[[nodiscard]] Vec3d operator/(Vec3d vector, double scalar);

[[nodiscard]] double dot(Vec3d lhs, Vec3d rhs);
[[nodiscard]] Vec3d cross(Vec3d lhs, Vec3d rhs);
[[nodiscard]] double length(Vec3d vector);
[[nodiscard]] Vec3d normalize(Vec3d vector);
[[nodiscard]] bool nearlyEqual(double lhs, double rhs, double tolerance = 1.0e-9);
[[nodiscard]] bool nearlyEqual(Vec3d lhs, Vec3d rhs, double tolerance = 1.0e-9);
[[nodiscard]] Vec3d transformPoint(const Mat4d& matrix, Vec3d point);
[[nodiscard]] Vec3d transformVector(const Mat4d& matrix, Vec3d vector);
[[nodiscard]] Mat4d invertAffine(const Mat4d& matrix);

}  // namespace measurement
