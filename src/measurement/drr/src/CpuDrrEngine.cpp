#include "measurement/drr/CpuDrrEngine.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace measurement {

namespace {

constexpr double kProjectionTolerance = 1.0e-6;

[[nodiscard]] bool isFinite(double value)
{
    return std::isfinite(value);
}

[[nodiscard]] bool isFinite(Vec3d value)
{
    return isFinite(value.x) && isFinite(value.y) && isFinite(value.z);
}

[[nodiscard]] bool dimensionsArePositive(Size3i dimensions)
{
    return dimensions.x > 0 && dimensions.y > 0 && dimensions.z > 0;
}

[[nodiscard]] bool volumeBoundsAreValid(const VolumeTransform& transform)
{
    return isFinite(transform.boundsMinPatientMm)
        && isFinite(transform.boundsMaxPatientMm)
        && transform.boundsMinPatientMm.x <= transform.boundsMaxPatientMm.x
        && transform.boundsMinPatientMm.y <= transform.boundsMaxPatientMm.y
        && transform.boundsMinPatientMm.z <= transform.boundsMaxPatientMm.z;
}

[[nodiscard]] Result<void> validateVolumeForDrr(const VolumeData& volume)
{
    if (!volume.image) {
        return Result<void>::failure({"DRR_VOLUME_EMPTY", "DRR render requires a volume image.", "", true});
    }

    if (!dimensionsArePositive(volume.image->dimensions()) || !volumeBoundsAreValid(volume.transform)) {
        return Result<void>::failure({"DRR_VOLUME_EMPTY", "DRR render requires a non-empty volume with valid patient bounds.", "", true});
    }

    return Result<void>::success();
}

[[nodiscard]] Result<void> validateSettings(const ProjectionParams& params, const DrrRenderSettings& settings)
{
    if (settings.width <= 0
        || settings.height <= 0
        || !isFinite(settings.stepMm)
        || settings.stepMm <= 0.0
        || !isFinite(params.pixelSpacingMm)
        || params.pixelSpacingMm <= 0.0
        || !isFinite(settings.windowCenter)
        || !isFinite(settings.windowWidth)
        || settings.windowWidth <= 0.0
        || !isFinite(settings.gamma)
        || settings.gamma <= 0.0
        || !isFinite(settings.huOffset)
        || !isFinite(settings.huScale)
        || settings.huScale <= 0.0) {
        return Result<void>::failure({"DRR_INVALID_SETTINGS", "DRR settings are invalid.", "", true});
    }

    return Result<void>::success();
}

[[nodiscard]] Result<void> validateProjection(const ProjectionParams& params)
{
    if (!isFinite(params.sourcePosPatientMm)
        || !isFinite(params.detectorCenterPatientMm)
        || !isFinite(params.detectorUPatientUnit)
        || !isFinite(params.detectorVPatientUnit)) {
        return Result<void>::failure({"DRR_INVALID_PROJECTION", "DRR projection contains non-finite geometry.", "", true});
    }

    const Vec3d sourceToDetector = params.detectorCenterPatientMm - params.sourcePosPatientMm;
    if (length(sourceToDetector) <= kProjectionTolerance) {
        return Result<void>::failure({"DRR_INVALID_PROJECTION", "DRR source and detector center must be separated.", "", true});
    }

    const double detectorULength = length(params.detectorUPatientUnit);
    const double detectorVLength = length(params.detectorVPatientUnit);
    if (detectorULength <= kProjectionTolerance || detectorVLength <= kProjectionTolerance) {
        return Result<void>::failure({"DRR_INVALID_PROJECTION", "DRR detector axes must be non-zero.", "", true});
    }

    const Vec3d detectorU = params.detectorUPatientUnit / detectorULength;
    const Vec3d detectorV = params.detectorVPatientUnit / detectorVLength;
    if (std::abs(dot(detectorU, detectorV)) > kProjectionTolerance) {
        return Result<void>::failure({"DRR_INVALID_PROJECTION", "DRR detector axes must be orthogonal.", "", true});
    }

    return Result<void>::success();
}

[[nodiscard]] float huToMu(int16_t hu, const DrrRenderSettings& settings)
{
    const double calibratedHu = static_cast<double>(hu) * settings.huScale + settings.huOffset;
    return static_cast<float>(std::max(calibratedHu + 1000.0, 0.0) / 1000.0);
}

[[nodiscard]] uint16_t mapIntegralToDisplay(float integral, const DrrRenderSettings& settings)
{
    const double lower = settings.windowCenter - settings.windowWidth * 0.5;
    const double normalized = std::clamp((static_cast<double>(integral) - lower) / settings.windowWidth, 0.0, 1.0);
    const double gammaCorrected = std::pow(normalized, 1.0 / settings.gamma);
    return static_cast<uint16_t>(std::clamp(gammaCorrected, 0.0, 1.0) * 65535.0);
}

[[nodiscard]] bool intersectAabb(Vec3d source, Vec3d direction, Vec3d boundsMin, Vec3d boundsMax, double& tEnter, double& tExit)
{
    tEnter = 0.0;
    tExit = std::numeric_limits<double>::max();

    const double sourceValues[3] = {source.x, source.y, source.z};
    const double directionValues[3] = {direction.x, direction.y, direction.z};
    const double minValues[3] = {boundsMin.x, boundsMin.y, boundsMin.z};
    const double maxValues[3] = {boundsMax.x, boundsMax.y, boundsMax.z};

    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(directionValues[axis]) < 1.0e-12) {
            if (sourceValues[axis] < minValues[axis] || sourceValues[axis] > maxValues[axis]) {
                return false;
            }
            continue;
        }

        double t0 = (minValues[axis] - sourceValues[axis]) / directionValues[axis];
        double t1 = (maxValues[axis] - sourceValues[axis]) / directionValues[axis];
        if (t0 > t1) {
            std::swap(t0, t1);
        }
        tEnter = std::max(tEnter, t0);
        tExit = std::min(tExit, t1);
        if (tExit < tEnter) {
            return false;
        }
    }
    return true;
}

[[nodiscard]] int clampIndex(double value, int upperExclusive)
{
    return std::clamp(static_cast<int>(std::llround(value)), 0, upperExclusive - 1);
}

[[nodiscard]] int16_t nearestHu(const VolumeData& volume, Vec3d patient)
{
    const Vec3d voxel = patientToVoxel(volume.transform, patient);
    const Size3i dims = volume.image->dimensions();
    const int i = clampIndex(voxel.x, dims.x);
    const int j = clampIndex(voxel.y, dims.y);
    const int k = clampIndex(voxel.z, dims.z);
    return volume.image->voxelHu(i, j, k);
}

}  // namespace

Result<void> CpuDrrEngine::setVolume(const VolumeData& volume)
{
    const auto validation = validateVolumeForDrr(volume);
    if (!validation.ok()) {
        return validation;
    }
    m_volume = volume;
    return Result<void>::success();
}

Result<DrrImage> CpuDrrEngine::render(const ProjectionParams& params, const DrrRenderSettings& settings) const
{
    if (!m_volume.image) {
        return Result<DrrImage>::failure({"DRR_VOLUME_EMPTY", "DRR render requires a volume image.", "", true});
    }

    const auto volumeValidation = validateVolumeForDrr(m_volume);
    if (!volumeValidation.ok()) {
        return Result<DrrImage>::failure(volumeValidation.error());
    }

    const auto settingsValidation = validateSettings(params, settings);
    if (!settingsValidation.ok()) {
        return Result<DrrImage>::failure(settingsValidation.error());
    }

    const auto projectionValidation = validateProjection(params);
    if (!projectionValidation.ok()) {
        return Result<DrrImage>::failure(projectionValidation.error());
    }

    DrrImage image;
    image.width = settings.width;
    image.height = settings.height;
    image.projection = params;
    if (settings.outputLineIntegral) {
        image.lineIntegral.resize(static_cast<size_t>(settings.width * settings.height), 0.0F);
    }
    image.displayImage.resize(static_cast<size_t>(settings.width * settings.height), 0U);

    const Vec3d detectorU = normalize(params.detectorUPatientUnit);
    const Vec3d detectorV = normalize(params.detectorVPatientUnit);

    for (int y = 0; y < settings.height; ++y) {
        for (int x = 0; x < settings.width; ++x) {
            const double offsetU = (static_cast<double>(x) + 0.5 - static_cast<double>(settings.width) * 0.5) * params.pixelSpacingMm;
            const double offsetV = (static_cast<double>(y) + 0.5 - static_cast<double>(settings.height) * 0.5) * params.pixelSpacingMm;
            const Vec3d pixelPos = params.detectorCenterPatientMm + detectorU * offsetU + detectorV * offsetV;
            const Vec3d rayDir = normalize(pixelPos - params.sourcePosPatientMm);

            double tEnter = 0.0;
            double tExit = 0.0;
            float integral = 0.0F;
            if (intersectAabb(params.sourcePosPatientMm, rayDir, m_volume.transform.boundsMinPatientMm, m_volume.transform.boundsMaxPatientMm, tEnter, tExit)) {
                for (double t = tEnter; t <= tExit; t += settings.stepMm) {
                    const Vec3d sample = params.sourcePosPatientMm + rayDir * t;
                    integral += huToMu(nearestHu(m_volume, sample), settings) * static_cast<float>(settings.stepMm);
                }
            }

            const size_t index = static_cast<size_t>(y * settings.width + x);
            if (settings.outputLineIntegral) {
                image.lineIntegral[index] = integral;
            }
            image.displayImage[index] = mapIntegralToDisplay(integral, settings);
        }
    }

    return Result<DrrImage>::success(std::move(image));
}

}  // namespace measurement
