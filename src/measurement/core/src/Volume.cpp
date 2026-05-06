#include "measurement/core/Volume.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <stdexcept>
#include <utility>

namespace measurement {

namespace {

constexpr double kDirectionTolerance = 1.0e-6;

[[nodiscard]] bool isFinite(Vec3d value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

[[nodiscard]] ErrorInfo invalidMetadata(std::string detail)
{
    return makeErrorInfo(
        std::string(kErrorVolumeInvalidMetadata),
        "Volume metadata is invalid.",
        std::move(detail),
        true);
}

[[nodiscard]] bool dimensionsArePositive(Size3i dimensions)
{
    return dimensions.x > 0 && dimensions.y > 0 && dimensions.z > 0;
}

[[nodiscard]] bool spacingIsPositiveAndFinite(Vec3d spacingMm)
{
    return isFinite(spacingMm) && spacingMm.x > 0.0 && spacingMm.y > 0.0 && spacingMm.z > 0.0;
}

[[nodiscard]] size_t voxelCount(Size3i dimensions)
{
    return static_cast<size_t>(dimensions.x)
        * static_cast<size_t>(dimensions.y)
        * static_cast<size_t>(dimensions.z);
}

[[nodiscard]] Vec3d minVec(Vec3d lhs, Vec3d rhs)
{
    return {std::min(lhs.x, rhs.x), std::min(lhs.y, rhs.y), std::min(lhs.z, rhs.z)};
}

[[nodiscard]] Vec3d maxVec(Vec3d lhs, Vec3d rhs)
{
    return {std::max(lhs.x, rhs.x), std::max(lhs.y, rhs.y), std::max(lhs.z, rhs.z)};
}

}  // namespace

DenseHuVolume::DenseHuVolume(Size3i dimensions, std::vector<int16_t> voxels)
    : m_dimensions(dimensions)
    , m_voxels(std::move(voxels))
{
}

Size3i DenseHuVolume::dimensions() const
{
    return m_dimensions;
}

int16_t DenseHuVolume::voxelHu(int i, int j, int k) const
{
    const size_t sliceSize = static_cast<size_t>(m_dimensions.x) * static_cast<size_t>(m_dimensions.y);
    const size_t index = static_cast<size_t>(k) * sliceSize
        + static_cast<size_t>(j) * static_cast<size_t>(m_dimensions.x)
        + static_cast<size_t>(i);
    return m_voxels.at(index);
}

const std::vector<int16_t>& DenseHuVolume::voxels() const
{
    return m_voxels;
}

Result<void> validateVolumeMetadata(const VolumeMetadata& metadata)
{
    if (!dimensionsArePositive(metadata.dimensions)) {
        return Result<void>::failure(invalidMetadata("dimensions must be positive."));
    }
    if (!spacingIsPositiveAndFinite(metadata.spacingMm)) {
        return Result<void>::failure(invalidMetadata("spacingMm must be finite and positive."));
    }
    if (!isFinite(metadata.originPatientMm)) {
        return Result<void>::failure(invalidMetadata("originPatientMm must be finite."));
    }
    if (!isFinite(metadata.rowDirectionPatient) || !isFinite(metadata.columnDirectionPatient) || !isFinite(metadata.sliceDirectionPatient)) {
        return Result<void>::failure(invalidMetadata("direction vectors must be finite."));
    }

    const double rowLength = length(metadata.rowDirectionPatient);
    const double columnLength = length(metadata.columnDirectionPatient);
    if (rowLength <= 0.0 || columnLength <= 0.0) {
        return Result<void>::failure(invalidMetadata("rowDirectionPatient and columnDirectionPatient must be non-zero."));
    }

    const Vec3d row = normalize(metadata.rowDirectionPatient);
    const Vec3d column = normalize(metadata.columnDirectionPatient);
    if (std::abs(dot(row, column)) > kDirectionTolerance) {
        return Result<void>::failure(invalidMetadata("rowDirectionPatient and columnDirectionPatient must be orthogonal."));
    }

    const double sliceLength = length(metadata.sliceDirectionPatient);
    if (sliceLength > 0.0) {
        const Vec3d slice = normalize(metadata.sliceDirectionPatient);
        if (std::abs(dot(row, slice)) > kDirectionTolerance || std::abs(dot(column, slice)) > kDirectionTolerance) {
            return Result<void>::failure(invalidMetadata("sliceDirectionPatient must be orthogonal to row and column directions."));
        }
    }

    if (!std::isfinite(metadata.rescaleSlope) || !std::isfinite(metadata.rescaleIntercept) || metadata.rescaleSlope == 0.0) {
        return Result<void>::failure(invalidMetadata("rescaleSlope must be non-zero and rescale parameters must be finite."));
    }
    if (metadata.minHu > metadata.maxHu) {
        return Result<void>::failure(invalidMetadata("minHu must be less than or equal to maxHu."));
    }

    return Result<void>::success();
}

Result<std::shared_ptr<DenseHuVolume>> makeDenseHuVolume(Size3i dimensions, std::vector<int16_t> voxels)
{
    if (!dimensionsArePositive(dimensions)) {
        return Result<std::shared_ptr<DenseHuVolume>>::failure(makeErrorInfo(
            std::string(kErrorVolumeImageSizeMismatch),
            "Dense HU volume dimensions are invalid.",
            "dimensions must be positive.",
            true));
    }

    const size_t expectedCount = voxelCount(dimensions);
    if (voxels.size() != expectedCount) {
        return Result<std::shared_ptr<DenseHuVolume>>::failure(makeErrorInfo(
            std::string(kErrorVolumeImageSizeMismatch),
            "Dense HU voxel count does not match dimensions.",
            "expected=" + std::to_string(expectedCount) + ", actual=" + std::to_string(voxels.size()),
            true));
    }

    return Result<std::shared_ptr<DenseHuVolume>>::success(
        std::make_shared<DenseHuVolume>(dimensions, std::move(voxels)));
}

Result<VolumeTransform> makeVolumeTransform(const VolumeMetadata& metadata)
{
    const auto validation = validateVolumeMetadata(metadata);
    if (!validation.ok()) {
        return Result<VolumeTransform>::failure(validation.error());
    }

    const Vec3d row = normalize(metadata.rowDirectionPatient);
    const Vec3d column = normalize(metadata.columnDirectionPatient);
    const Vec3d slice = length(metadata.sliceDirectionPatient) > 0.0
        ? normalize(metadata.sliceDirectionPatient)
        : normalize(cross(row, column));

    Mat4d voxelToPatient = Mat4d::identity();
    voxelToPatient.at(0, 0) = row.x * metadata.spacingMm.x;
    voxelToPatient.at(1, 0) = row.y * metadata.spacingMm.x;
    voxelToPatient.at(2, 0) = row.z * metadata.spacingMm.x;

    voxelToPatient.at(0, 1) = column.x * metadata.spacingMm.y;
    voxelToPatient.at(1, 1) = column.y * metadata.spacingMm.y;
    voxelToPatient.at(2, 1) = column.z * metadata.spacingMm.y;

    voxelToPatient.at(0, 2) = slice.x * metadata.spacingMm.z;
    voxelToPatient.at(1, 2) = slice.y * metadata.spacingMm.z;
    voxelToPatient.at(2, 2) = slice.z * metadata.spacingMm.z;

    voxelToPatient.at(0, 3) = metadata.originPatientMm.x;
    voxelToPatient.at(1, 3) = metadata.originPatientMm.y;
    voxelToPatient.at(2, 3) = metadata.originPatientMm.z;

    VolumeTransform transform;
    transform.voxelToPatient = voxelToPatient;
    try {
        transform.patientToVoxel = invertAffine(voxelToPatient);
    } catch (const std::runtime_error& error) {
        return Result<VolumeTransform>::failure(makeErrorInfo(
            std::string(kErrorVolumeTransformNotInvertible),
            "Volume transform is not invertible.",
            error.what(),
            false));
    }

    const std::array<Vec3d, 8> corners = {
        Vec3d{0.0, 0.0, 0.0},
        Vec3d{static_cast<double>(metadata.dimensions.x - 1), 0.0, 0.0},
        Vec3d{0.0, static_cast<double>(metadata.dimensions.y - 1), 0.0},
        Vec3d{0.0, 0.0, static_cast<double>(metadata.dimensions.z - 1)},
        Vec3d{static_cast<double>(metadata.dimensions.x - 1), static_cast<double>(metadata.dimensions.y - 1), 0.0},
        Vec3d{static_cast<double>(metadata.dimensions.x - 1), 0.0, static_cast<double>(metadata.dimensions.z - 1)},
        Vec3d{0.0, static_cast<double>(metadata.dimensions.y - 1), static_cast<double>(metadata.dimensions.z - 1)},
        Vec3d{static_cast<double>(metadata.dimensions.x - 1), static_cast<double>(metadata.dimensions.y - 1), static_cast<double>(metadata.dimensions.z - 1)},
    };

    Vec3d boundsMin{
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
    };
    Vec3d boundsMax{
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest(),
    };

    for (const Vec3d corner : corners) {
        const Vec3d patient = transformPoint(voxelToPatient, corner);
        boundsMin = minVec(boundsMin, patient);
        boundsMax = maxVec(boundsMax, patient);
    }

    transform.boundsMinPatientMm = boundsMin;
    transform.boundsMaxPatientMm = boundsMax;
    return Result<VolumeTransform>::success(transform);
}

Vec3d voxelToPatient(const VolumeTransform& transform, Vec3d voxel)
{
    return transformPoint(transform.voxelToPatient, voxel);
}

Vec3d patientToVoxel(const VolumeTransform& transform, Vec3d patient)
{
    return transformPoint(transform.patientToVoxel, patient);
}

}  // namespace measurement
