#include "measurement/vtk/VtkPhysicsAdapter.h"

#include <cmath>

namespace measurement {

Result<ProjectionParams> VtkPhysicsAdapter::buildProjectionParams(const VtkCameraFrame& frame) const
{
    if (frame.sidMm <= 0.0) {
        return Result<ProjectionParams>::failure({"XRAY_SID_INVALID", "SID must be positive.", "", true});
    }

    const Mat4d worldToActor = invertAffine(frame.actorToWorld);
    const Vec3d source = transformPoint(worldToActor, frame.cameraPositionWorld);
    const Vec3d focal = transformPoint(worldToActor, frame.focalPointWorld);
    const Vec3d viewUp = normalize(transformVector(worldToActor, frame.viewUpWorld));
    const Vec3d ray = focal - source;
    const double sod = length(ray);
    if (sod <= 0.0) {
        return Result<ProjectionParams>::failure({"XRAY_CAMERA_INVALID", "Camera source and focal point must differ.", "", true});
    }

    const Vec3d rayDir = ray / sod;
    Vec3d detectorU = normalize(cross(viewUp, rayDir));
    if (length(detectorU) < 1.0e-6) {
        const Vec3d fallback = std::abs(rayDir.x) < 0.9 ? Vec3d{1.0, 0.0, 0.0} : Vec3d{0.0, 1.0, 0.0};
        detectorU = normalize(cross(fallback, rayDir));
    }
    const Vec3d detectorV = normalize(cross(detectorU, rayDir));

    ProjectionParams params;
    params.sourcePosPatientMm = source;
    params.detectorCenterPatientMm = source + rayDir * frame.sidMm;
    params.detectorUPatientUnit = detectorU;
    params.detectorVPatientUnit = detectorV;
    params.sidMm = frame.sidMm;
    params.sodMm = sod;
    return Result<ProjectionParams>::success(params);
}

}  // namespace measurement
