#include "measurement/core/MeasurementVisibility.h"

#include <algorithm>
#include <cmath>

namespace measurement::measurement_visibility {
namespace {

constexpr double kEpsilon = 1.0e-7;
constexpr double kPi = 3.14159265358979323846;

[[nodiscard]] bool inBand(double distance, double halfThickness)
{
    return std::abs(distance) <= halfThickness + kEpsilon;
}

[[nodiscard]] bool intervalIntersectsBand(double first, double second, double halfThickness)
{
    const double minDistance = std::min(first, second);
    const double maxDistance = std::max(first, second);
    return maxDistance >= -halfThickness - kEpsilon && minDistance <= halfThickness + kEpsilon;
}

[[nodiscard]] bool crossesBandFromOutside(double first, double second, double halfThickness)
{
    return (first < -halfThickness - kEpsilon && second > halfThickness + kEpsilon)
        || (second < -halfThickness - kEpsilon && first > halfThickness + kEpsilon);
}

[[nodiscard]] MeasurementPlane shiftedPlane(const MeasurementPlane& plane, double signedOffset)
{
    MeasurementPlane shifted = plane;
    shifted.center = plane.center + plane.normal * signedOffset;
    return shifted;
}

[[nodiscard]] bool samePoint(Vec3d first, Vec3d second)
{
    return length(first - second) <= kEpsilon;
}

void appendUnique(std::vector<Vec3d>& points, Vec3d point)
{
    const auto it = std::find_if(points.begin(), points.end(), [point](Vec3d existing) {
        return samePoint(existing, point);
    });
    if (it == points.end()) {
        points.push_back(point);
    }
}

[[nodiscard]] std::vector<Vec3d> sectionIntersections(
    Vec3d first,
    Vec3d second,
    const MeasurementPlane& slice,
    double halfThickness)
{
    std::vector<Vec3d> points;

    if (auto point = shiftedPlane(slice, -halfThickness).intersectSegment(first, second)) {
        appendUnique(points, *point);
    }
    if (auto point = shiftedPlane(slice, halfThickness).intersectSegment(first, second)) {
        appendUnique(points, *point);
    }

    return points;
}

[[nodiscard]] double angleDeg(Vec3d first, Vec3d second)
{
    const double denominator = length(first) * length(second);
    if (denominator <= kEpsilon) {
        return 0.0;
    }
    const double cosine = std::clamp(dot(normalize(first), normalize(second)), -1.0, 1.0);
    return std::acos(std::abs(cosine)) * 180.0 / kPi;
}

[[nodiscard]] bool isOblique(MeasurementViewType viewType)
{
    return viewType == MeasurementViewType::Oblique;
}

}  // namespace

MeasurementVisibilityResult evaluateDistance(Vec3d first, Vec3d second, const MeasurementPlane& slice)
{
    const double halfThickness = std::max(0.0, slice.thicknessMm * 0.5);
    const double firstDistance = slice.signedDistance(first);
    const double secondDistance = slice.signedDistance(second);

    MeasurementVisibilityResult result;
    if (inBand(firstDistance, halfThickness) && inBand(secondDistance, halfThickness)) {
        result.level = MeasurementVisibilityLevel::FullDisplay;
        result.fullWorldPointsPatientMm = {first, second};
        return result;
    }

    if (intervalIntersectsBand(firstDistance, secondDistance, halfThickness)
        && crossesBandFromOutside(firstDistance, secondDistance, halfThickness)) {
        result.sectionWorldPointsPatientMm = sectionIntersections(first, second, slice, halfThickness);
        if (result.sectionWorldPointsPatientMm.size() == 2) {
            result.level = MeasurementVisibilityLevel::SectionIndicator;
            return result;
        }
        result.sectionWorldPointsPatientMm.clear();
    }

    return result;
}

MeasurementVisibilityResult evaluateAngle(
    Vec3d line1Start,
    Vec3d line1End,
    Vec3d line2Start,
    Vec3d line2End,
    const MeasurementPlane& slice)
{
    const double halfThickness = std::max(0.0, slice.thicknessMm * 0.5);
    const double firstDistance = slice.signedDistance(line1Start);
    const double secondDistance = slice.signedDistance(line1End);
    const double thirdDistance = slice.signedDistance(line2Start);
    const double fourthDistance = slice.signedDistance(line2End);

    MeasurementVisibilityResult result;
    result.type = MeasurementType::Angle;

    if (inBand(firstDistance, halfThickness) && inBand(secondDistance, halfThickness)
        && inBand(thirdDistance, halfThickness) && inBand(fourthDistance, halfThickness)) {
        result.level = MeasurementVisibilityLevel::FullDisplay;
        result.fullWorldPointsPatientMm = {line1Start, line1End, line2Start, line2End};
        return result;
    }

    const MeasurementVisibilityResult firstEdge = evaluateDistance(line1Start, line1End, slice);
    const MeasurementVisibilityResult secondEdge = evaluateDistance(line2Start, line2End, slice);
    if (firstEdge.level != MeasurementVisibilityLevel::Hidden || secondEdge.level != MeasurementVisibilityLevel::Hidden) {
        result.level = MeasurementVisibilityLevel::SectionIndicator;
        for (Vec3d point : firstEdge.sectionWorldPointsPatientMm) {
            appendUnique(result.sectionWorldPointsPatientMm, point);
        }
        for (Vec3d point : secondEdge.sectionWorldPointsPatientMm) {
            appendUnique(result.sectionWorldPointsPatientMm, point);
        }
    }

    return result;
}

void applyObliqueDirectionFilter(
    MeasurementVisibilityResult& result,
    Vec3d createdNormal,
    const MeasurementPlane& currentSlice,
    MeasurementViewType currentViewType,
    double angleToleranceDeg)
{
    if (result.level == MeasurementVisibilityLevel::Hidden || !isOblique(currentViewType)) {
        return;
    }

    if (length(createdNormal) <= kEpsilon || length(currentSlice.normal) <= kEpsilon) {
        return;
    }

    if (angleDeg(createdNormal, currentSlice.normal) > angleToleranceDeg) {
        result.level = MeasurementVisibilityLevel::Hidden;
        result.fullWorldPointsPatientMm.clear();
        result.sectionWorldPointsPatientMm.clear();
    }
}

MeasurementVisibilityResult evaluate(
    const MeasurementAnnotation& annotation,
    const MeasurementPlane& slice,
    MeasurementViewType currentViewType)
{
    MeasurementVisibilityResult result;
    if (annotation.type == MeasurementType::Distance && annotation.worldPointsPatientMm.size() == 2) {
        result = evaluateDistance(annotation.worldPointsPatientMm[0], annotation.worldPointsPatientMm[1], slice);
    } else if (annotation.type == MeasurementType::Angle && annotation.worldPointsPatientMm.size() == 4) {
        result = evaluateAngle(
            annotation.worldPointsPatientMm[0],
            annotation.worldPointsPatientMm[1],
            annotation.worldPointsPatientMm[2],
            annotation.worldPointsPatientMm[3],
            slice);
    }

    result.id = annotation.id;
    result.type = annotation.type;
    result.label = annotation.label;
    result.measurementText = annotation.measurementText();
    result.displayText = annotation.displayText();
    result.selected = annotation.selected;

    applyObliqueDirectionFilter(result, annotation.createdPlane.normal, slice, currentViewType);
    return result;
}

}  // namespace measurement::measurement_visibility
