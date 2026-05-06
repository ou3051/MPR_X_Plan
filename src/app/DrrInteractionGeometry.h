#pragma once

#include "measurement/core/Geometry.h"
#include "measurement/core/Xray.h"

#include <optional>

namespace measurement_app {

struct DrrDetectorPoint {
    double x = 0.0;
    double y = 0.0;
};

struct DrrDetectorLine {
    DrrDetectorPoint head;
    DrrDetectorPoint tail;
};

struct DrrInteractionRay {
    measurement::Vec3d originPatientMm;
    measurement::Vec3d directionPatientUnit;
};

struct DrrRayClosestPoint {
    measurement::Vec3d pointPatientMm;
    double distanceMm = 0.0;
};

[[nodiscard]] std::optional<DrrDetectorPoint> projectPatientToDetectorPixel(
    const measurement::ProjectionParams& projection,
    measurement::Vec3d patientPointMm);

[[nodiscard]] std::optional<DrrInteractionRay> detectorPixelToPatientRay(
    const measurement::ProjectionParams& projection,
    DrrDetectorPoint detectorPoint);

[[nodiscard]] std::optional<DrrRayClosestPoint> closestPointBetweenRays(
    const DrrInteractionRay& first,
    const DrrInteractionRay& second);

[[nodiscard]] std::optional<measurement::Vec3d> rayPlaneIntersection(
    const DrrInteractionRay& ray,
    measurement::Vec3d planePointPatientMm,
    measurement::Vec3d planeNormalPatientUnit);

[[nodiscard]] measurement::Vec3d closestPointOnRay(
    const DrrInteractionRay& ray,
    measurement::Vec3d patientPointMm);

[[nodiscard]] std::optional<measurement::Vec3d> raySphereIntersectionNearDirection(
    const DrrInteractionRay& ray,
    measurement::Vec3d sphereCenterPatientMm,
    double radiusMm,
    measurement::Vec3d preferredDirectionPatientUnit);

[[nodiscard]] double distancePointToSegmentPx(
    DrrDetectorPoint point,
    DrrDetectorPoint segmentStart,
    DrrDetectorPoint segmentEnd);

}  // namespace measurement_app
