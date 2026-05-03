#pragma once

#include "measurement/core/Geometry.h"

#include <cstdint>
#include <vector>

namespace measurement {

enum class XrayPreset {
    AP,
    LAT,
    Oblique,
    Custom
};

struct ProjectionParams {
    Vec3d sourcePosPatientMm;
    Vec3d detectorCenterPatientMm;
    Vec3d detectorUPatientUnit{1.0, 0.0, 0.0};
    Vec3d detectorVPatientUnit{0.0, 1.0, 0.0};
    double pixelSpacingMm = 0.5;
    int detectorWidth = 512;
    int detectorHeight = 512;
    double primaryAngleDeg = 0.0;
    double secondaryAngleDeg = 0.0;
    double sidMm = 1000.0;
    double sodMm = 700.0;
};

struct XrayView {
    XrayPreset preset = XrayPreset::AP;
    ProjectionParams projection;
    double windowCenter = 0.0;
    double windowWidth = 1.0;
    bool showInstrumentOverlay = true;
};

struct DrrRenderSettings {
    int width = 512;
    int height = 512;
    double stepMm = 0.5;
    bool outputLineIntegral = true;
    double windowCenter = 0.0;
    double windowWidth = 1.0;
    double gamma = 1.0;
};

struct DrrImage {
    int width = 0;
    int height = 0;
    std::vector<float> lineIntegral;
    std::vector<uint16_t> displayImage;
    ProjectionParams projection;
    uint64_t frameId = 0;
};

}  // namespace measurement
