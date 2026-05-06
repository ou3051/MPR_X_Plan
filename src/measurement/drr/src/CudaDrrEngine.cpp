#include "measurement/drr/CudaDrrEngine.h"

#if MEASUREMENT_HAVE_CUDA
#include "CudaDrrKernel.h"
#endif

#include <cmath>
#include <cstdint>
#include <string>

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

[[nodiscard]] Result<void> validateVolumeForCudaDrr(const VolumeData& volume)
{
    if (!volume.image) {
        return Result<void>::failure({"CUDA_DRR_VOLUME_EMPTY", "CUDA DRR render requires a volume image.", "", true});
    }
    if (!dimensionsArePositive(volume.image->dimensions())) {
        return Result<void>::failure({"CUDA_DRR_VOLUME_EMPTY", "CUDA DRR render requires a non-empty volume image.", "", true});
    }
    if (!isFinite(volume.transform.boundsMinPatientMm)
        || !isFinite(volume.transform.boundsMaxPatientMm)
        || volume.transform.boundsMinPatientMm.x > volume.transform.boundsMaxPatientMm.x
        || volume.transform.boundsMinPatientMm.y > volume.transform.boundsMaxPatientMm.y
        || volume.transform.boundsMinPatientMm.z > volume.transform.boundsMaxPatientMm.z) {
        return Result<void>::failure({"CUDA_DRR_VOLUME_EMPTY", "CUDA DRR render requires valid patient bounds.", "", true});
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
        return Result<void>::failure({"CUDA_DRR_INVALID_SETTINGS", "CUDA DRR settings are invalid.", "", true});
    }
    return Result<void>::success();
}

[[nodiscard]] Result<void> validateProjection(const ProjectionParams& params)
{
    if (!isFinite(params.sourcePosPatientMm)
        || !isFinite(params.detectorCenterPatientMm)
        || !isFinite(params.detectorUPatientUnit)
        || !isFinite(params.detectorVPatientUnit)) {
        return Result<void>::failure({"CUDA_DRR_INVALID_PROJECTION", "CUDA DRR projection contains non-finite geometry.", "", true});
    }
    if (length(params.detectorCenterPatientMm - params.sourcePosPatientMm) <= kProjectionTolerance) {
        return Result<void>::failure({"CUDA_DRR_INVALID_PROJECTION", "CUDA DRR source and detector center must be separated.", "", true});
    }
    const double detectorULength = length(params.detectorUPatientUnit);
    const double detectorVLength = length(params.detectorVPatientUnit);
    if (detectorULength <= kProjectionTolerance || detectorVLength <= kProjectionTolerance) {
        return Result<void>::failure({"CUDA_DRR_INVALID_PROJECTION", "CUDA DRR detector axes must be non-zero.", "", true});
    }
    const Vec3d detectorU = params.detectorUPatientUnit / detectorULength;
    const Vec3d detectorV = params.detectorVPatientUnit / detectorVLength;
    if (std::abs(dot(detectorU, detectorV)) > kProjectionTolerance) {
        return Result<void>::failure({"CUDA_DRR_INVALID_PROJECTION", "CUDA DRR detector axes must be orthogonal.", "", true});
    }
    return Result<void>::success();
}

[[nodiscard]] std::string volumeSignature(const VolumeData& volume)
{
    if (!volume.image) {
        return {};
    }

    const Size3i dimensions = volume.image->dimensions();
    // The GPU cache is keyed by immutable-ish volume identity plus geometry. The shared image
    // pointer distinguishes synthetic/test volumes that do not carry DICOM hashes yet.
    return volume.studyUid + "|"
        + volume.seriesUid + "|"
        + volume.dataHash + "|"
        + volume.patientPositionCode + "|"
        + std::to_string(reinterpret_cast<std::uintptr_t>(volume.image.get())) + "|"
        + std::to_string(dimensions.x) + "x"
        + std::to_string(dimensions.y) + "x"
        + std::to_string(dimensions.z) + "|"
        + std::to_string(volume.metadata.spacingMm.x) + ","
        + std::to_string(volume.metadata.spacingMm.y) + ","
        + std::to_string(volume.metadata.spacingMm.z) + "|"
        + std::to_string(volume.metadata.originPatientMm.x) + ","
        + std::to_string(volume.metadata.originPatientMm.y) + ","
        + std::to_string(volume.metadata.originPatientMm.z) + "|"
        + std::to_string(volume.metadata.rowDirectionPatient.x) + ","
        + std::to_string(volume.metadata.rowDirectionPatient.y) + ","
        + std::to_string(volume.metadata.rowDirectionPatient.z) + "|"
        + std::to_string(volume.metadata.columnDirectionPatient.x) + ","
        + std::to_string(volume.metadata.columnDirectionPatient.y) + ","
        + std::to_string(volume.metadata.columnDirectionPatient.z) + "|"
        + std::to_string(volume.metadata.sliceDirectionPatient.x) + ","
        + std::to_string(volume.metadata.sliceDirectionPatient.y) + ","
        + std::to_string(volume.metadata.sliceDirectionPatient.z) + "|"
        + std::to_string(volume.transform.boundsMinPatientMm.x) + ","
        + std::to_string(volume.transform.boundsMinPatientMm.y) + ","
        + std::to_string(volume.transform.boundsMinPatientMm.z) + "|"
        + std::to_string(volume.transform.boundsMaxPatientMm.x) + ","
        + std::to_string(volume.transform.boundsMaxPatientMm.y) + ","
        + std::to_string(volume.transform.boundsMaxPatientMm.z);
}

}  // namespace

struct CudaDrrEngine::Impl {
#if MEASUREMENT_HAVE_CUDA
    struct DeviceVolumeDeleter {
        void operator()(CudaDrrDeviceVolume* deviceVolume) const noexcept
        {
            destroyCudaDrrDeviceVolume(deviceVolume);
        }
    };

    std::unique_ptr<CudaDrrDeviceVolume, DeviceVolumeDeleter> deviceVolume;
    std::string deviceVolumeSignature;
#endif
};

CudaDrrEngine::CudaDrrEngine()
    : m_impl(std::make_unique<Impl>())
{
}

CudaDrrEngine::~CudaDrrEngine() = default;
CudaDrrEngine::CudaDrrEngine(CudaDrrEngine&&) noexcept = default;
CudaDrrEngine& CudaDrrEngine::operator=(CudaDrrEngine&&) noexcept = default;

Result<void> CudaDrrEngine::setVolume(const VolumeData& volume)
{
    const auto validation = validateVolumeForCudaDrr(volume);
    if (!validation.ok()) {
        return validation;
    }

#if MEASUREMENT_HAVE_CUDA
    const std::string signature = volumeSignature(volume);
    if (m_impl->deviceVolume != nullptr && m_impl->deviceVolumeSignature == signature) {
        // Metadata may still have been copied by the caller, but the GPU buffer already matches
        // the underlying voxels and fixed patient->voxel transform.
        m_volume = volume;
        return Result<void>::success();
    }

    const auto deviceVolume = createCudaDrrDeviceVolume(volume);
    if (!deviceVolume.ok()) {
        return Result<void>::failure(deviceVolume.error());
    }

    // Swap only after upload succeeds so a transient CUDA allocation failure leaves the previous
    // engine state intact for callers that may retry or fall back to CPU.
    m_impl->deviceVolume.reset(deviceVolume.value());
    m_impl->deviceVolumeSignature = signature;
#endif

    m_volume = volume;
    return Result<void>::success();
}

Result<DrrImage> CudaDrrEngine::render(const ProjectionParams& params, const DrrRenderSettings& settings) const
{
    const auto volumeValidation = validateVolumeForCudaDrr(m_volume);
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

#if MEASUREMENT_HAVE_CUDA
    if (m_impl->deviceVolume == nullptr) {
        return Result<DrrImage>::failure({
            "CUDA_DRR_VOLUME_EMPTY",
            "CUDA DRR render requires an uploaded GPU volume.",
            "",
            true,
        });
    }
    return renderCudaDrr(*m_impl->deviceVolume, params, settings);
#else
    (void)params;
    (void)settings;
    return Result<DrrImage>::failure({
        "CUDA_DRR_DISABLED",
        "CUDA DRR is disabled because CUDA Toolkit was not found or MPR_ENABLE_CUDA_DRR is OFF.",
        "",
        true,
    });
#endif
}

}  // namespace measurement
