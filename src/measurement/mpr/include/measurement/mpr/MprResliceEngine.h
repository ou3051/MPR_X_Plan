#pragma once

#include "measurement/core/Geometry.h"
#include "measurement/core/Result.h"
#include "measurement/core/Volume.h"

#include <optional>
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

struct MprSliceFrame {
    Vec3d originPatientMm;
    Vec3d horizontalPatientUnit{1.0, 0.0, 0.0};
    Vec3d verticalPatientUnit{0.0, 1.0, 0.0};
    Vec3d normalPatientUnit{0.0, 0.0, 1.0};
};

struct MprViewState {
    MprPlane plane = MprPlane::Axial;
    Vec3d crosshairPatientMm;
    std::optional<MprSliceFrame> obliqueFrame;
    double zoom = 1.0;
    Vec3d pan;
    double windowCenterHu = 400.0;
    double windowWidthHu = 2000.0;
};

struct MprSliceRequest {
    int outputWidth = 512;
    int outputHeight = 512;
    double pixelSpacingMm = 1.0;
};

struct MprResliceParameters {
    MprSliceFrame frame;
    MprSliceRequest request;
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
[[nodiscard]] Result<MprSliceFrame> defaultSliceFrame(const VolumeMetadata& metadata, MprPlane plane, Vec3d originPatientMm);
[[nodiscard]] Result<MprResliceParameters> buildMprResliceParameters(
    const VolumeData& volume,
    const MprViewState& state,
    const MprSliceRequest& request);

}  // namespace measurement
