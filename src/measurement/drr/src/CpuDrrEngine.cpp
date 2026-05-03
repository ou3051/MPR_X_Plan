#include "measurement/drr/CpuDrrEngine.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace measurement {

namespace {

[[nodiscard]] float huToMu(int16_t hu)
{
    return static_cast<float>(std::max(static_cast<int>(hu) + 1000, 0)) / 1000.0F;
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
    if (!volume.image) {
        return Result<void>::failure({"DRR_VOLUME_EMPTY", "DRR render requires a volume image.", "", true});
    }
    m_volume = volume;
    return Result<void>::success();
}

Result<DrrImage> CpuDrrEngine::render(const ProjectionParams& params, const DrrRenderSettings& settings) const
{
    if (!m_volume.image) {
        return Result<DrrImage>::failure({"DRR_VOLUME_EMPTY", "DRR render requires a volume image.", "", true});
    }
    if (settings.width <= 0 || settings.height <= 0 || settings.stepMm <= 0.0 || params.pixelSpacingMm <= 0.0) {
        return Result<DrrImage>::failure({"DRR_INVALID_SETTINGS", "DRR settings are invalid.", "", true});
    }

    DrrImage image;
    image.width = settings.width;
    image.height = settings.height;
    image.projection = params;
    image.lineIntegral.resize(static_cast<size_t>(settings.width * settings.height), 0.0F);
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
                    integral += huToMu(nearestHu(m_volume, sample)) * static_cast<float>(settings.stepMm);
                }
            }

            const size_t index = static_cast<size_t>(y * settings.width + x);
            image.lineIntegral[index] = integral;
            const double lower = settings.windowCenter - settings.windowWidth * 0.5;
            const double normalized = settings.windowWidth > 0.0 ? std::clamp((static_cast<double>(integral) - lower) / settings.windowWidth, 0.0, 1.0) : 0.0;
            image.displayImage[index] = static_cast<uint16_t>(normalized * 65535.0);
        }
    }

    return Result<DrrImage>::success(std::move(image));
}

}  // namespace measurement
