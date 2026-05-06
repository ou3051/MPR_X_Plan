#include "measurement/core/MeasurementAnnotation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace measurement {
namespace {

constexpr double kMinimumSegmentLengthMm = 1.0e-6;
constexpr double kPi = 3.14159265358979323846;

[[nodiscard]] std::string formatValue(double value, const char* suffix)
{
    char buffer[64] = {};
    std::snprintf(buffer, sizeof(buffer), "%.1f %s", value, suffix);
    return buffer;
}

[[nodiscard]] double distanceBetween(Vec3d first, Vec3d second)
{
    return length(second - first);
}

[[nodiscard]] double angleBetween(Vec3d first, Vec3d second)
{
    const double denominator = length(first) * length(second);
    if (denominator <= kMinimumSegmentLengthMm * kMinimumSegmentLengthMm) {
        return 0.0;
    }

    const double cosine = std::clamp(std::abs(dot(first, second) / denominator), -1.0, 1.0);
    return std::acos(cosine) * 180.0 / kPi;
}

}  // namespace

MeasurementId::MeasurementId(std::int64_t value)
    : m_value(value)
{
}

std::int64_t MeasurementId::value() const
{
    return m_value;
}

bool MeasurementId::isValid() const
{
    return m_value >= 0;
}

bool MeasurementId::operator==(MeasurementId other) const
{
    return m_value == other.m_value;
}

bool MeasurementId::operator!=(MeasurementId other) const
{
    return !(*this == other);
}

bool MeasurementId::operator<(MeasurementId other) const
{
    return m_value < other.m_value;
}

double MeasurementPlane::signedDistance(Vec3d point) const
{
    return dot(point - center, normal);
}

std::optional<Vec3d> MeasurementPlane::intersectSegment(Vec3d start, Vec3d end) const
{
    const double startDistance = signedDistance(start);
    const double endDistance = signedDistance(end);
    const double denominator = endDistance - startDistance;
    if (std::abs(denominator) < 1.0e-9) {
        return std::nullopt;
    }

    const double t = -startDistance / denominator;
    if (t < 0.0 || t > 1.0) {
        return std::nullopt;
    }

    return start + (end - start) * t;
}

MeasurementAnnotation MeasurementAnnotation::makeDistance(Vec3d first, Vec3d second)
{
    MeasurementAnnotation annotation;
    annotation.type = MeasurementType::Distance;
    annotation.worldPointsPatientMm = {first, second};
    (void)annotation.recalculateValue();
    return annotation;
}

MeasurementAnnotation MeasurementAnnotation::makeAngle(
    Vec3d line1Start,
    Vec3d line1End,
    Vec3d line2Start,
    Vec3d line2End)
{
    MeasurementAnnotation annotation;
    annotation.type = MeasurementType::Angle;
    annotation.worldPointsPatientMm = {line1Start, line1End, line2Start, line2End};
    if (!annotation.recalculateValue()) {
        annotation.worldPointsPatientMm.clear();
        annotation.value = 0.0;
    }
    return annotation;
}

std::optional<MeasurementAnnotation> MeasurementAnnotation::tryMakeAngle(
    Vec3d line1Start,
    Vec3d line1End,
    Vec3d line2Start,
    Vec3d line2End)
{
    MeasurementAnnotation annotation = makeAngle(line1Start, line1End, line2Start, line2End);
    if (annotation.worldPointsPatientMm.size() != 4) {
        return std::nullopt;
    }
    return annotation;
}

bool MeasurementAnnotation::updateDistanceEndpoint(int pointIndex, Vec3d point)
{
    if (type != MeasurementType::Distance || worldPointsPatientMm.size() != 2 || (pointIndex != 0 && pointIndex != 1)) {
        return false;
    }

    worldPointsPatientMm[static_cast<size_t>(pointIndex)] = point;
    return recalculateValue();
}

bool MeasurementAnnotation::updateAngleEndpoint(int pointIndex, Vec3d point)
{
    if (type != MeasurementType::Angle || worldPointsPatientMm.size() != 4 || pointIndex < 0 || pointIndex > 3) {
        return false;
    }

    const Vec3d previous = worldPointsPatientMm[static_cast<size_t>(pointIndex)];
    worldPointsPatientMm[static_cast<size_t>(pointIndex)] = point;
    if (recalculateValue()) {
        return true;
    }

    worldPointsPatientMm[static_cast<size_t>(pointIndex)] = previous;
    (void)recalculateValue();
    return false;
}

bool MeasurementAnnotation::recalculateValue()
{
    if (type == MeasurementType::Distance && worldPointsPatientMm.size() == 2) {
        value = distanceBetween(worldPointsPatientMm[0], worldPointsPatientMm[1]);
        return true;
    }

    if (type == MeasurementType::Angle
        && worldPointsPatientMm.size() == 4
        && length(worldPointsPatientMm[1] - worldPointsPatientMm[0]) > kMinimumSegmentLengthMm
        && length(worldPointsPatientMm[3] - worldPointsPatientMm[2]) > kMinimumSegmentLengthMm) {
        value = angleBetween(
            worldPointsPatientMm[1] - worldPointsPatientMm[0],
            worldPointsPatientMm[3] - worldPointsPatientMm[2]);
        return true;
    }

    return false;
}

std::string MeasurementAnnotation::measurementText() const
{
    return type == MeasurementType::Angle ? formatValue(value, "deg") : formatValue(value, "mm");
}

std::string MeasurementAnnotation::displayText() const
{
    return label.empty() ? measurementText() : label;
}

std::optional<Vec3d> MeasurementAnnotation::anchorWorldPoint() const
{
    if (type == MeasurementType::Distance && worldPointsPatientMm.size() == 2) {
        return (worldPointsPatientMm[0] + worldPointsPatientMm[1]) * 0.5;
    }

    if (type == MeasurementType::Angle && worldPointsPatientMm.size() == 4) {
        const Vec3d firstMidpoint = (worldPointsPatientMm[0] + worldPointsPatientMm[1]) * 0.5;
        const Vec3d secondMidpoint = (worldPointsPatientMm[2] + worldPointsPatientMm[3]) * 0.5;
        return (firstMidpoint + secondMidpoint) * 0.5;
    }

    return std::nullopt;
}

}  // namespace measurement
