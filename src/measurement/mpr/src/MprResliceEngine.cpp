#include "measurement/mpr/MprResliceEngine.h"

#include <cmath>
#include <string>

namespace measurement {

namespace {

constexpr double kMprTolerance = 1.0e-6;

[[nodiscard]] bool isFinite(Vec3d value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

[[nodiscard]] ErrorInfo mprError(const char* code, const char* message, std::string detail = {})
{
    return makeErrorInfo(code, message, std::move(detail), true);
}

[[nodiscard]] bool dimensionsArePositive(Size3i dimensions)
{
    return dimensions.x > 0 && dimensions.y > 0 && dimensions.z > 0;
}

[[nodiscard]] bool containsVoxelPoint(Size3i dimensions, Vec3d voxel)
{
    return voxel.x >= -kMprTolerance && voxel.y >= -kMprTolerance && voxel.z >= -kMprTolerance
        && voxel.x <= static_cast<double>(dimensions.x - 1) + kMprTolerance
        && voxel.y <= static_cast<double>(dimensions.y - 1) + kMprTolerance
        && voxel.z <= static_cast<double>(dimensions.z - 1) + kMprTolerance;
}

[[nodiscard]] Result<MprSliceFrame> normalizeFrame(MprSliceFrame frame)
{
    if (!isFinite(frame.originPatientMm)
        || !isFinite(frame.horizontalPatientUnit)
        || !isFinite(frame.verticalPatientUnit)
        || !isFinite(frame.normalPatientUnit)) {
        return Result<MprSliceFrame>::failure(mprError(
            "MPR_FRAME_INVALID",
            "MPR slice frame is invalid.",
            "frame vectors must be finite."));
    }

    if (length(frame.horizontalPatientUnit) <= kMprTolerance
        || length(frame.verticalPatientUnit) <= kMprTolerance
        || length(frame.normalPatientUnit) <= kMprTolerance) {
        return Result<MprSliceFrame>::failure(mprError(
            "MPR_FRAME_INVALID",
            "MPR slice frame is invalid.",
            "frame axes must be non-zero."));
    }

    const Vec3d horizontal = normalize(frame.horizontalPatientUnit);
    Vec3d vertical = normalize(frame.verticalPatientUnit);
    const Vec3d declaredNormal = normalize(frame.normalPatientUnit);
    const Vec3d derivedNormal = normalize(cross(horizontal, vertical));
    if (length(derivedNormal) <= kMprTolerance) {
        return Result<MprSliceFrame>::failure(mprError(
            "MPR_FRAME_INVALID",
            "MPR slice frame is invalid.",
            "horizontal and vertical axes must not be collinear."));
    }
    if (std::abs(dot(horizontal, vertical)) > kMprTolerance
        || std::abs(dot(horizontal, declaredNormal)) > kMprTolerance
        || std::abs(dot(vertical, declaredNormal)) > kMprTolerance) {
        return Result<MprSliceFrame>::failure(mprError(
            "MPR_FRAME_INVALID",
            "MPR slice frame is invalid.",
            "frame axes must be orthogonal."));
    }
    if (dot(derivedNormal, declaredNormal) < 1.0 - kMprTolerance) {
        return Result<MprSliceFrame>::failure(mprError(
            "MPR_FRAME_INVALID",
            "MPR slice frame is invalid.",
            "normal must match horizontal x vertical."));
    }

    frame.horizontalPatientUnit = horizontal;
    frame.verticalPatientUnit = vertical;
    frame.normalPatientUnit = declaredNormal;
    return Result<MprSliceFrame>::success(frame);
}

}  // namespace

Vec3d planeNormalPatient(const VolumeMetadata& metadata, MprPlane plane)
{
    switch (plane) {
    case MprPlane::Axial:
        return normalize(metadata.sliceDirectionPatient);
    case MprPlane::Sagittal:
        return normalize(metadata.rowDirectionPatient);
    case MprPlane::Coronal:
        return normalize(metadata.columnDirectionPatient);
    }
    return {};
}

Result<MprSliceFrame> defaultSliceFrame(const VolumeMetadata& metadata, MprPlane plane, Vec3d originPatientMm)
{
    const auto metadataValidation = validateVolumeMetadata(metadata);
    if (!metadataValidation.ok()) {
        return Result<MprSliceFrame>::failure(mprError(
            "MPR_VOLUME_METADATA_INVALID",
            "MPR reslice requires valid volume metadata.",
            metadataValidation.error().detail));
    }

    const Vec3d row = normalize(metadata.rowDirectionPatient);
    const Vec3d column = normalize(metadata.columnDirectionPatient);
    const Vec3d slice = length(metadata.sliceDirectionPatient) > 0.0
        ? normalize(metadata.sliceDirectionPatient)
        : normalize(cross(row, column));

    MprSliceFrame frame;
    frame.originPatientMm = originPatientMm;
    switch (plane) {
    case MprPlane::Axial:
        frame.horizontalPatientUnit = row;
        frame.verticalPatientUnit = column;
        frame.normalPatientUnit = slice;
        break;
    case MprPlane::Sagittal:
        frame.horizontalPatientUnit = column;
        frame.verticalPatientUnit = slice;
        frame.normalPatientUnit = row;
        break;
    case MprPlane::Coronal:
        frame.horizontalPatientUnit = slice;
        frame.verticalPatientUnit = row;
        frame.normalPatientUnit = column;
        break;
    }

    return normalizeFrame(frame);
}

Result<MprResliceParameters> buildMprResliceParameters(
    const VolumeData& volume,
    const MprViewState& state,
    const MprSliceRequest& request)
{
    if (!volume.image) {
        return Result<MprResliceParameters>::failure(mprError(
            "MPR_VOLUME_EMPTY",
            "MPR reslice requires a volume image."));
    }

    const auto metadataValidation = validateVolumeMetadata(volume.metadata);
    if (!metadataValidation.ok() || !dimensionsArePositive(volume.metadata.dimensions)) {
        return Result<MprResliceParameters>::failure(mprError(
            "MPR_VOLUME_METADATA_INVALID",
            "MPR reslice requires valid volume metadata.",
            metadataValidation.ok() ? "dimensions must be positive." : metadataValidation.error().detail));
    }

    if (!isFinite(state.crosshairPatientMm)) {
        return Result<MprResliceParameters>::failure(mprError(
            "MPR_FRAME_INVALID",
            "MPR crosshair is invalid.",
            "crosshairPatientMm must be finite."));
    }
    if (!std::isfinite(state.zoom) || state.zoom <= 0.0) {
        return Result<MprResliceParameters>::failure(mprError(
            "MPR_FRAME_INVALID",
            "MPR zoom is invalid.",
            "zoom must be finite and positive."));
    }
    if (!isFinite(state.pan)) {
        return Result<MprResliceParameters>::failure(mprError(
            "MPR_FRAME_INVALID",
            "MPR pan is invalid.",
            "pan must be finite."));
    }
    if (request.outputWidth <= 0 || request.outputHeight <= 0 || !std::isfinite(request.pixelSpacingMm) || request.pixelSpacingMm <= 0.0) {
        return Result<MprResliceParameters>::failure(mprError(
            "MPR_OUTPUT_INVALID",
            "MPR output request is invalid.",
            "output dimensions and pixel spacing must be positive."));
    }

    const Vec3d crosshairVoxel = patientToVoxel(volume.transform, state.crosshairPatientMm);
    if (!isFinite(crosshairVoxel) || !containsVoxelPoint(volume.metadata.dimensions, crosshairVoxel)) {
        return Result<MprResliceParameters>::failure(mprError(
            "MPR_CROSSHAIR_OUT_OF_BOUNDS",
            "MPR crosshair is outside the volume.",
            "crosshairPatientMm must map inside the image extent."));
    }

    Result<MprSliceFrame> frameResult = state.obliqueFrame.has_value()
        ? normalizeFrame(*state.obliqueFrame)
        : defaultSliceFrame(volume.metadata, state.plane, state.crosshairPatientMm);
    if (!frameResult.ok()) {
        return Result<MprResliceParameters>::failure(frameResult.error());
    }

    MprSliceFrame frame = frameResult.value();
    frame.originPatientMm = frame.originPatientMm
        + frame.horizontalPatientUnit * state.pan.x
        + frame.verticalPatientUnit * state.pan.y;

    const Vec3d frameOriginVoxel = patientToVoxel(volume.transform, frame.originPatientMm);
    if (!isFinite(frameOriginVoxel) || !containsVoxelPoint(volume.metadata.dimensions, frameOriginVoxel)) {
        return Result<MprResliceParameters>::failure(mprError(
            "MPR_CROSSHAIR_OUT_OF_BOUNDS",
            "MPR slice origin is outside the volume.",
            "reslice origin must map inside the image extent."));
    }

    MprSliceRequest normalizedRequest = request;
    normalizedRequest.pixelSpacingMm = request.pixelSpacingMm / state.zoom;

    return Result<MprResliceParameters>::success({frame, normalizedRequest});
}

Result<MprSliceImage> MprResliceEngine::reslice(const VolumeData& volume, const MprViewState& state) const
{
    const auto parameters = buildMprResliceParameters(volume, state, {});
    if (!parameters.ok()) {
        return Result<MprSliceImage>::failure(parameters.error());
    }

    return Result<MprSliceImage>::failure(mprError(
        "MPR_RESLICE_ADAPTER_REQUIRED",
        "MPR pixel resampling is provided by the VTK vtkImageReslice adapter.",
        "Core MPR parameters are valid, but this target intentionally has no VTK dependency."));
}

}  // namespace measurement
