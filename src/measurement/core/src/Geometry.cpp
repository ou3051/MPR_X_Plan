#include "measurement/core/Geometry.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace measurement {

Mat4d Mat4d::identity()
{
    Mat4d matrix;
    matrix.at(0, 0) = 1.0;
    matrix.at(1, 1) = 1.0;
    matrix.at(2, 2) = 1.0;
    matrix.at(3, 3) = 1.0;
    return matrix;
}

double Mat4d::at(int row, int column) const
{
    return values[static_cast<size_t>(row * 4 + column)];
}

double& Mat4d::at(int row, int column)
{
    return values[static_cast<size_t>(row * 4 + column)];
}

Vec3d operator+(Vec3d lhs, Vec3d rhs)
{
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vec3d operator-(Vec3d lhs, Vec3d rhs)
{
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3d operator*(Vec3d vector, double scalar)
{
    return {vector.x * scalar, vector.y * scalar, vector.z * scalar};
}

Vec3d operator*(double scalar, Vec3d vector)
{
    return vector * scalar;
}

Vec3d operator/(Vec3d vector, double scalar)
{
    return {vector.x / scalar, vector.y / scalar, vector.z / scalar};
}

double dot(Vec3d lhs, Vec3d rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

Vec3d cross(Vec3d lhs, Vec3d rhs)
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    };
}

double length(Vec3d vector)
{
    return std::sqrt(dot(vector, vector));
}

Vec3d normalize(Vec3d vector)
{
    const double vectorLength = length(vector);
    if (vectorLength <= 0.0) {
        return {};
    }
    return vector / vectorLength;
}

bool nearlyEqual(double lhs, double rhs, double tolerance)
{
    return std::abs(lhs - rhs) <= tolerance;
}

bool nearlyEqual(Vec3d lhs, Vec3d rhs, double tolerance)
{
    return nearlyEqual(lhs.x, rhs.x, tolerance)
        && nearlyEqual(lhs.y, rhs.y, tolerance)
        && nearlyEqual(lhs.z, rhs.z, tolerance);
}

Vec3d transformPoint(const Mat4d& matrix, Vec3d point)
{
    const double x = matrix.at(0, 0) * point.x + matrix.at(0, 1) * point.y + matrix.at(0, 2) * point.z + matrix.at(0, 3);
    const double y = matrix.at(1, 0) * point.x + matrix.at(1, 1) * point.y + matrix.at(1, 2) * point.z + matrix.at(1, 3);
    const double z = matrix.at(2, 0) * point.x + matrix.at(2, 1) * point.y + matrix.at(2, 2) * point.z + matrix.at(2, 3);
    const double w = matrix.at(3, 0) * point.x + matrix.at(3, 1) * point.y + matrix.at(3, 2) * point.z + matrix.at(3, 3);
    if (w != 0.0 && w != 1.0) {
        return {x / w, y / w, z / w};
    }
    return {x, y, z};
}

Vec3d transformVector(const Mat4d& matrix, Vec3d vector)
{
    return {
        matrix.at(0, 0) * vector.x + matrix.at(0, 1) * vector.y + matrix.at(0, 2) * vector.z,
        matrix.at(1, 0) * vector.x + matrix.at(1, 1) * vector.y + matrix.at(1, 2) * vector.z,
        matrix.at(2, 0) * vector.x + matrix.at(2, 1) * vector.y + matrix.at(2, 2) * vector.z,
    };
}

Mat4d invertAffine(const Mat4d& matrix)
{
    const double a00 = matrix.at(0, 0);
    const double a01 = matrix.at(0, 1);
    const double a02 = matrix.at(0, 2);
    const double a10 = matrix.at(1, 0);
    const double a11 = matrix.at(1, 1);
    const double a12 = matrix.at(1, 2);
    const double a20 = matrix.at(2, 0);
    const double a21 = matrix.at(2, 1);
    const double a22 = matrix.at(2, 2);

    const double c00 = a11 * a22 - a12 * a21;
    const double c01 = -(a10 * a22 - a12 * a20);
    const double c02 = a10 * a21 - a11 * a20;
    const double determinant = a00 * c00 + a01 * c01 + a02 * c02;

    if (std::abs(determinant) < 1.0e-12) {
        throw std::runtime_error("Affine matrix is not invertible");
    }

    const double invDet = 1.0 / determinant;
    Mat4d inverse = Mat4d::identity();
    inverse.at(0, 0) = c00 * invDet;
    inverse.at(0, 1) = (a02 * a21 - a01 * a22) * invDet;
    inverse.at(0, 2) = (a01 * a12 - a02 * a11) * invDet;
    inverse.at(1, 0) = c01 * invDet;
    inverse.at(1, 1) = (a00 * a22 - a02 * a20) * invDet;
    inverse.at(1, 2) = (a02 * a10 - a00 * a12) * invDet;
    inverse.at(2, 0) = c02 * invDet;
    inverse.at(2, 1) = (a01 * a20 - a00 * a21) * invDet;
    inverse.at(2, 2) = (a00 * a11 - a01 * a10) * invDet;

    const Vec3d translation{matrix.at(0, 3), matrix.at(1, 3), matrix.at(2, 3)};
    const Vec3d inverseTranslation = transformVector(inverse, translation) * -1.0;
    inverse.at(0, 3) = inverseTranslation.x;
    inverse.at(1, 3) = inverseTranslation.y;
    inverse.at(2, 3) = inverseTranslation.z;
    return inverse;
}

}  // namespace measurement
