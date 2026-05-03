#include "measurement/core/Volume.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace measurement {

namespace {

[[nodiscard]] bool validMetadata(const VolumeMetadata& metadata)
{
    return metadata.dimensions.x > 0 && metadata.dimensions.y > 0 && metadata.dimensions.z > 0
        && metadata.spacingMm.x > 0.0 && metadata.spacingMm.y > 0.0 && metadata.spacingMm.z > 0.0
        && length(metadata.rowDirectionPatient) > 0.0
        && length(metadata.columnDirectionPatient) > 0.0;
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

Result<VolumeTransform> makeVolumeTransform(const VolumeMetadata& metadata)
{
    if (!validMetadata(metadata)) {
        return Result<VolumeTransform>::failure({
            "VOLUME_INVALID_METADATA",
            "Volume metadata is invalid.",
            "Dimensions, spacing, row direction, and column direction must be valid.",
            true,
        });
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
        return Result<VolumeTransform>::failure({
            "VOLUME_TRANSFORM_NOT_INVERTIBLE",
            "Volume transform is not invertible.",
            error.what(),
            false,
        });
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
