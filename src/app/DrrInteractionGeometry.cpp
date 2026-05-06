#include "DrrInteractionGeometry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace measurement_app {
namespace {

constexpr double kEpsilon = 1.0e-9;

[[nodiscard]] bool isFinite(double value)
{
    return std::isfinite(value);
}

[[nodiscard]] bool isFinite(measurement::Vec3d value)
{
    return isFinite(value.x) && isFinite(value.y) && isFinite(value.z);
}

[[nodiscard]] std::optional<std::pair<measurement::Vec3d, measurement::Vec3d>> detectorAxes(
    const measurement::ProjectionParams& projection)
{
    const measurement::Vec3d u = measurement::normalize(projection.detectorUPatientUnit);
    const measurement::Vec3d v = measurement::normalize(projection.detectorVPatientUnit);
    if (!isFinite(u) || !isFinite(v) || measurement::length(u) <= kEpsilon || measurement::length(v) <= kEpsilon) {
        return std::nullopt;
    }
    if (std::abs(measurement::dot(u, v)) > 1.0e-5) {
        return std::nullopt;
    }
    return std::make_pair(u, v);
}

}  // namespace

std::optional<DrrDetectorPoint> projectPatientToDetectorPixel(
    const measurement::ProjectionParams& projection,
    measurement::Vec3d patientPointMm)
{
    if (!isFinite(patientPointMm)
        || !isFinite(projection.sourcePosPatientMm)
        || !isFinite(projection.detectorCenterPatientMm)
        || !isFinite(projection.pixelSpacingMm)
        || projection.pixelSpacingMm <= 0.0
        || projection.detectorWidth <= 0
        || projection.detectorHeight <= 0) {
        return std::nullopt;
    }

    const auto axes = detectorAxes(projection);
    if (!axes.has_value()) {
        return std::nullopt;
    }

    const measurement::Vec3d rayDirection = measurement::normalize(patientPointMm - projection.sourcePosPatientMm);
    const measurement::Vec3d detectorNormal = measurement::normalize(measurement::cross(axes->first, axes->second));
    const double denominator = measurement::dot(rayDirection, detectorNormal);
    if (std::abs(denominator) <= kEpsilon) {
        return std::nullopt;
    }
    const double t = measurement::dot(projection.detectorCenterPatientMm - projection.sourcePosPatientMm, detectorNormal)
        / denominator;
    if (t < 0.0 || !std::isfinite(t)) {
        return std::nullopt;
    }

    const measurement::Vec3d detectorIntersection = projection.sourcePosPatientMm + rayDirection * t;
    const measurement::Vec3d delta = detectorIntersection - projection.detectorCenterPatientMm;
    return DrrDetectorPoint{
        measurement::dot(delta, axes->first) / projection.pixelSpacingMm
            + static_cast<double>(projection.detectorWidth) * 0.5
            - 0.5,
        measurement::dot(delta, axes->second) / projection.pixelSpacingMm
            + static_cast<double>(projection.detectorHeight) * 0.5
            - 0.5,
    };
}

std::optional<DrrInteractionRay> detectorPixelToPatientRay(
    const measurement::ProjectionParams& projection,
    DrrDetectorPoint detectorPoint)
{
    if (!isFinite(detectorPoint.x)
        || !isFinite(detectorPoint.y)
        || !isFinite(projection.sourcePosPatientMm)
        || !isFinite(projection.detectorCenterPatientMm)
        || !isFinite(projection.pixelSpacingMm)
        || projection.pixelSpacingMm <= 0.0
        || projection.detectorWidth <= 0
        || projection.detectorHeight <= 0) {
        return std::nullopt;
    }

    const auto axes = detectorAxes(projection);
    if (!axes.has_value()) {
        return std::nullopt;
    }

    const double offsetU = (detectorPoint.x + 0.5 - static_cast<double>(projection.detectorWidth) * 0.5)
        * projection.pixelSpacingMm;
    const double offsetV = (detectorPoint.y + 0.5 - static_cast<double>(projection.detectorHeight) * 0.5)
        * projection.pixelSpacingMm;
    const measurement::Vec3d detectorPointPatient =
        projection.detectorCenterPatientMm + axes->first * offsetU + axes->second * offsetV;
    const measurement::Vec3d direction = measurement::normalize(detectorPointPatient - projection.sourcePosPatientMm);
    if (!isFinite(direction) || measurement::length(direction) <= kEpsilon) {
        return std::nullopt;
    }
    return DrrInteractionRay{projection.sourcePosPatientMm, direction};
}

std::optional<DrrRayClosestPoint> closestPointBetweenRays(
    const DrrInteractionRay& first,
    const DrrInteractionRay& second)
{
    if (!isFinite(first.originPatientMm)
        || !isFinite(first.directionPatientUnit)
        || !isFinite(second.originPatientMm)
        || !isFinite(second.directionPatientUnit)) {
        return std::nullopt;
    }

    const measurement::Vec3d d1 = measurement::normalize(first.directionPatientUnit);
    const measurement::Vec3d d2 = measurement::normalize(second.directionPatientUnit);
    const measurement::Vec3d r = first.originPatientMm - second.originPatientMm;
    const double b = measurement::dot(d1, d2);
    const double d = measurement::dot(d1, r);
    const double e = measurement::dot(d2, r);
    const double denominator = 1.0 - b * b;
    if (std::abs(denominator) <= kEpsilon) {
        return std::nullopt;
    }

    const double t1 = std::max(0.0, (b * e - d) / denominator);
    const double t2 = std::max(0.0, (e - b * d) / denominator);
    const measurement::Vec3d p1 = first.originPatientMm + d1 * t1;
    const measurement::Vec3d p2 = second.originPatientMm + d2 * t2;
    return DrrRayClosestPoint{(p1 + p2) * 0.5, measurement::length(p1 - p2)};
}

std::optional<measurement::Vec3d> rayPlaneIntersection(
    const DrrInteractionRay& ray,
    measurement::Vec3d planePointPatientMm,
    measurement::Vec3d planeNormalPatientUnit)
{
    if (!isFinite(ray.originPatientMm)
        || !isFinite(ray.directionPatientUnit)
        || !isFinite(planePointPatientMm)
        || !isFinite(planeNormalPatientUnit)) {
        return std::nullopt;
    }

    const measurement::Vec3d direction = measurement::normalize(ray.directionPatientUnit);
    const measurement::Vec3d normal = measurement::normalize(planeNormalPatientUnit);
    const double denominator = measurement::dot(direction, normal);
    if (std::abs(denominator) <= kEpsilon) {
        return std::nullopt;
    }

    const double t = measurement::dot(planePointPatientMm - ray.originPatientMm, normal) / denominator;
    if (t < 0.0 || !std::isfinite(t)) {
        return std::nullopt;
    }
    return ray.originPatientMm + direction * t;
}

measurement::Vec3d closestPointOnRay(const DrrInteractionRay& ray, measurement::Vec3d patientPointMm)
{
    const measurement::Vec3d direction = measurement::normalize(ray.directionPatientUnit);
    const double t = std::max(0.0, measurement::dot(patientPointMm - ray.originPatientMm, direction));
    return ray.originPatientMm + direction * t;
}

std::optional<measurement::Vec3d> raySphereIntersectionNearDirection(
    const DrrInteractionRay& ray,
    measurement::Vec3d sphereCenterPatientMm,
    double radiusMm,
    measurement::Vec3d preferredDirectionPatientUnit)
{
    if (!isFinite(ray.originPatientMm)
        || !isFinite(ray.directionPatientUnit)
        || !isFinite(sphereCenterPatientMm)
        || !std::isfinite(radiusMm)
        || radiusMm <= 0.0) {
        return std::nullopt;
    }

    const measurement::Vec3d direction = measurement::normalize(ray.directionPatientUnit);
    const measurement::Vec3d originToCenter = ray.originPatientMm - sphereCenterPatientMm;
    const double b = 2.0 * measurement::dot(direction, originToCenter);
    const double c = measurement::dot(originToCenter, originToCenter) - radiusMm * radiusMm;
    const double discriminant = b * b - 4.0 * c;
    if (discriminant < 0.0) {
        return std::nullopt;
    }

    const double sqrtDiscriminant = std::sqrt(discriminant);
    const std::array<double, 2> candidates{(-b - sqrtDiscriminant) * 0.5, (-b + sqrtDiscriminant) * 0.5};
    const measurement::Vec3d preferred = measurement::normalize(preferredDirectionPatientUnit);
    double bestScore = -(std::numeric_limits<double>::max)();
    std::optional<measurement::Vec3d> bestPoint;
    for (double t : candidates) {
        if (t < 0.0 || !std::isfinite(t)) {
            continue;
        }
        const measurement::Vec3d point = ray.originPatientMm + direction * t;
        const measurement::Vec3d candidateDirection = measurement::normalize(point - sphereCenterPatientMm);
        const double score = isFinite(preferred) && measurement::length(preferred) > kEpsilon
            ? measurement::dot(candidateDirection, preferred)
            : t;
        if (score > bestScore) {
            bestScore = score;
            bestPoint = point;
        }
    }
    return bestPoint;
}

double distancePointToSegmentPx(
    DrrDetectorPoint point,
    DrrDetectorPoint segmentStart,
    DrrDetectorPoint segmentEnd)
{
    const double dx = segmentEnd.x - segmentStart.x;
    const double dy = segmentEnd.y - segmentStart.y;
    const double len2 = dx * dx + dy * dy;
    if (len2 <= kEpsilon) {
        return std::hypot(point.x - segmentStart.x, point.y - segmentStart.y);
    }
    const double t = std::clamp(
        ((point.x - segmentStart.x) * dx + (point.y - segmentStart.y) * dy) / len2,
        0.0,
        1.0);
    const double closestX = segmentStart.x + dx * t;
    const double closestY = segmentStart.y + dy * t;
    return std::hypot(point.x - closestX, point.y - closestY);
}

}  // namespace measurement_app
