#pragma once

#include "measurement/core/Geometry.h"
#include "measurement/core/Result.h"

#include <cstdint>
#include <memory>
#include <string>

namespace measurement {

struct VolumeMetadata {
    Size3i dimensions;
    Vec3d spacingMm;
    Vec3d originPatientMm;
    Vec3d rowDirectionPatient;
    Vec3d columnDirectionPatient;
    Vec3d sliceDirectionPatient;
    double rescaleSlope = 1.0;
    double rescaleIntercept = 0.0;
    int minHu = 0;
    int maxHu = 0;
};

struct VolumeTransform {
    Mat4d voxelToPatient = Mat4d::identity();
    Mat4d patientToVoxel = Mat4d::identity();
    Vec3d boundsMinPatientMm;
    Vec3d boundsMaxPatientMm;
};

class IImageVolume {
public:
    virtual ~IImageVolume() = default;
    [[nodiscard]] virtual Size3i dimensions() const = 0;
    [[nodiscard]] virtual int16_t voxelHu(int i, int j, int k) const = 0;
};

struct VolumeData {
    VolumeMetadata metadata;
    VolumeTransform transform;
    std::shared_ptr<IImageVolume> image;
    std::string sourceFolder;
    std::string seriesUid;
    std::string studyUid;
    std::string dataHash;
};

[[nodiscard]] Result<VolumeTransform> makeVolumeTransform(const VolumeMetadata& metadata);
[[nodiscard]] Vec3d voxelToPatient(const VolumeTransform& transform, Vec3d voxel);
[[nodiscard]] Vec3d patientToVoxel(const VolumeTransform& transform, Vec3d patient);

}  // namespace measurement
