#pragma once

#include "measurement/core/Geometry.h"
#include "measurement/core/Result.h"
#include "measurement/core/Volume.h"

#include <vector>

namespace measurement {

enum class MprPlane {
    Axial,
    Sagittal,
    Coronal
};

struct ScreenPoint {
    double x = 0.0;
    double y = 0.0;
};

struct MprViewState {
    MprPlane plane = MprPlane::Axial;
    Vec3d crosshairPatientMm;
    double zoom = 1.0;
    Vec3d pan;
    double windowCenterHu = 400.0;
    double windowWidthHu = 2000.0;
};

struct MprSliceImage {
    int width = 0;
    int height = 0;
    std::vector<int16_t> huPixels;
};

class MprResliceEngine {
public:
    [[nodiscard]] Result<MprSliceImage> reslice(const VolumeData& volume, const MprViewState& state) const;
};

[[nodiscard]] Vec3d planeNormalPatient(const VolumeMetadata& metadata, MprPlane plane);

}  // namespace measurement
