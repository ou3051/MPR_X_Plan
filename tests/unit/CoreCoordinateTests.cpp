#include "measurement/core/Volume.h"

#include <gtest/gtest.h>

namespace measurement {
namespace {

TEST(CoreCoordinateTests, MakeVolumeTransform_RoundTripsVoxelAndPatient)
{
    VolumeMetadata metadata;
    metadata.dimensions = {10, 20, 30};
    metadata.spacingMm = {0.5, 0.75, 1.25};
    metadata.originPatientMm = {10.0, 20.0, 30.0};
    metadata.rowDirectionPatient = {1.0, 0.0, 0.0};
    metadata.columnDirectionPatient = {0.0, 1.0, 0.0};
    metadata.sliceDirectionPatient = {0.0, 0.0, 1.0};

    const auto transform = makeVolumeTransform(metadata);
    ASSERT_TRUE(transform.ok()) << transform.error().message;

    const Vec3d voxel{4.0, 5.0, 6.0};
    const Vec3d patient = voxelToPatient(transform.value(), voxel);
    EXPECT_TRUE(nearlyEqual(patient, Vec3d{12.0, 23.75, 37.5}));
    EXPECT_TRUE(nearlyEqual(patientToVoxel(transform.value(), patient), voxel));
}

TEST(CoreCoordinateTests, MakeVolumeTransform_RejectsInvalidSpacing)
{
    VolumeMetadata metadata;
    metadata.dimensions = {10, 20, 30};
    metadata.spacingMm = {0.0, 0.75, 1.25};
    metadata.rowDirectionPatient = {1.0, 0.0, 0.0};
    metadata.columnDirectionPatient = {0.0, 1.0, 0.0};

    const auto transform = makeVolumeTransform(metadata);
    EXPECT_FALSE(transform.ok());
    EXPECT_EQ(transform.error().code, "VOLUME_INVALID_METADATA");
}

}  // namespace
}  // namespace measurement
