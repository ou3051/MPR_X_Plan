#pragma once

#include "measurement/core/MeasurementAnnotation.h"

namespace measurement::measurement_visibility {

[[nodiscard]] MeasurementVisibilityResult evaluateDistance(Vec3d first, Vec3d second, const MeasurementPlane& slice);
[[nodiscard]] MeasurementVisibilityResult evaluateAngle(
    Vec3d line1Start,
    Vec3d line1End,
    Vec3d line2Start,
    Vec3d line2End,
    const MeasurementPlane& slice);
void applyObliqueDirectionFilter(
    MeasurementVisibilityResult& result,
    Vec3d createdNormal,
    const MeasurementPlane& currentSlice,
    MeasurementViewType currentViewType,
    double angleToleranceDeg = 15.0);
[[nodiscard]] MeasurementVisibilityResult evaluate(
    const MeasurementAnnotation& annotation,
    const MeasurementPlane& slice,
    MeasurementViewType currentViewType);

}  // namespace measurement::measurement_visibility
