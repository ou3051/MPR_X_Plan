#pragma once

#include "measurement/core/Geometry.h"
#include "measurement/core/Result.h"
#include "measurement/core/Xray.h"

namespace measurement {

struct VtkCameraFrame {
    Vec3d cameraPositionWorld;
    Vec3d focalPointWorld;
    Vec3d viewUpWorld{0.0, 1.0, 0.0};
    Mat4d actorToWorld = Mat4d::identity();
    double sidMm = 1000.0;
};

class VtkPhysicsAdapter {
public:
    [[nodiscard]] Result<ProjectionParams> buildProjectionParams(const VtkCameraFrame& frame) const;
};

}  // namespace measurement
