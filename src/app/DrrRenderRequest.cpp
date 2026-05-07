#include "DrrRenderRequest.h"

#include <algorithm>
#include <cmath>

namespace measurement_app {
namespace {

constexpr double kDefaultPixelSpacingMm = 1.0;
constexpr int kMaxDrrDetectorSamples = 4096;

[[nodiscard]] bool isFinite(measurement::Vec3d value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

}  // namespace

bool buildDrrRenderRequest(
    const measurement::VolumeData* volume,
    measurement::XrayPreset preset,
    const DrrUiSettings& uiSettings,
    measurement::ProjectionParams& projection,
    measurement::DrrRenderSettings& renderSettings)
{
    if (volume == nullptr || !volume->image) {
        return false;
    }

    const measurement::Vec3d boundsMin = volume->transform.boundsMinPatientMm;
    const measurement::Vec3d boundsMax = volume->transform.boundsMaxPatientMm;
    if (!isFinite(boundsMin) || !isFinite(boundsMax)) {
        return false;
    }

    const measurement::Vec3d center = (boundsMin + boundsMax) * 0.5;
    const measurement::Vec3d extent = boundsMax - boundsMin;
    const double maxExtent = std::max({extent.x, extent.y, extent.z, 1.0});
    const double sidMm = std::max(uiSettings.sidMm, 2.0);
    const double sodMm = std::clamp(uiSettings.sodMm, 1.0, sidMm - 1.0e-3);

    projection = {};
    projection.sourcePosPatientMm = center;
    projection.detectorCenterPatientMm = center;
    projection.sidMm = sidMm;
    projection.sodMm = sodMm;

    if (preset == measurement::XrayPreset::LAT) {
        projection.sourcePosPatientMm = center + measurement::Vec3d{-sodMm, 0.0, 0.0};
        projection.detectorCenterPatientMm = center + measurement::Vec3d{sidMm - sodMm, 0.0, 0.0};
        projection.detectorUPatientUnit = {0.0, 1.0, 0.0};
        projection.detectorVPatientUnit = {0.0, 0.0, 1.0};
        projection.primaryAngleDeg = 90.0;
    } else {
        projection.sourcePosPatientMm = center + measurement::Vec3d{0.0, -sodMm, 0.0};
        projection.detectorCenterPatientMm = center + measurement::Vec3d{0.0, sidMm - sodMm, 0.0};
        projection.detectorUPatientUnit = {1.0, 0.0, 0.0};
        projection.detectorVPatientUnit = {0.0, 0.0, 1.0};
        projection.primaryAngleDeg = 0.0;
    }

    const double detectorWidthMm = std::max(uiSettings.detectorWidthMm, 1.0);
    const double detectorHeightMm = std::max(uiSettings.detectorHeightMm, 1.0);
    double detectorPixelSpacingMm = uiSettings.pixelSpacingMm > 0.0
        ? uiSettings.pixelSpacingMm
        : kDefaultPixelSpacingMm;
    if (!std::isfinite(detectorPixelSpacingMm) || detectorPixelSpacingMm <= 0.0) {
        detectorPixelSpacingMm = kDefaultPixelSpacingMm;
    }
    detectorPixelSpacingMm = std::max({
        detectorPixelSpacingMm,
        detectorWidthMm / static_cast<double>(kMaxDrrDetectorSamples),
        detectorHeightMm / static_cast<double>(kMaxDrrDetectorSamples),
    });
    renderSettings.width = std::max(1, static_cast<int>(std::llround(detectorWidthMm / detectorPixelSpacingMm)));
    renderSettings.height = std::max(1, static_cast<int>(std::llround(detectorHeightMm / detectorPixelSpacingMm)));
    renderSettings.stepMm = std::max(uiSettings.rayStepMm, 1.0e-6);
    renderSettings.outputLineIntegral = true;
    renderSettings.windowCenter = uiSettings.windowCenter > 0.0 ? uiSettings.windowCenter : maxExtent * 0.55;
    renderSettings.windowWidth = uiSettings.windowWidth > 0.0 ? uiSettings.windowWidth : std::max(maxExtent * 1.1, 1.0);
    renderSettings.gamma = std::max(uiSettings.gamma, 1.0e-6);
    renderSettings.huOffset = uiSettings.huOffset;
    renderSettings.huScale = std::max(uiSettings.huScale, 1.0e-6);

    projection.detectorWidth = renderSettings.width;
    projection.detectorHeight = renderSettings.height;
    projection.pixelSpacingMm = detectorPixelSpacingMm;
    if (!std::isfinite(projection.pixelSpacingMm) || projection.pixelSpacingMm <= 0.0) {
        projection.pixelSpacingMm = 1.0;
    }

    return true;
}

}  // namespace measurement_app
