#pragma once

#include "measurement/core/Volume.h"
#include "measurement/core/Xray.h"

namespace measurement_app {

struct DrrUiSettings {
    double sidMm = 1000.0;
    double sodMm = 700.0;
    double detectorWidthMm = 320.0;
    double detectorHeightMm = 240.0;
    double pixelSpacingMm = 0.0;
    double rayStepMm = 1.0;
    double windowCenter = 0.0;
    double windowWidth = 0.0;
    double gamma = 1.0;
    double huOffset = 0.0;
    double huScale = 1.0;
};

[[nodiscard]] bool buildDrrRenderRequest(
    const measurement::VolumeData* volume,
    measurement::XrayPreset preset,
    const DrrUiSettings& uiSettings,
    measurement::ProjectionParams& projection,
    measurement::DrrRenderSettings& renderSettings);

}  // namespace measurement_app
