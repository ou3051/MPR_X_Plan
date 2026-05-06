#include "measurement/core/Volume.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace measurement {
namespace {

[[nodiscard]] VolumeMetadata makeValidMetadata()
{
    VolumeMetadata metadata;
    metadata.dimensions = {10, 20, 30};
    metadata.spacingMm = {0.5, 0.75, 1.25};
    metadata.originPatientMm = {10.0, 20.0, 30.0};
    metadata.rowDirectionPatient = {1.0, 0.0, 0.0};
    metadata.columnDirectionPatient = {0.0, 1.0, 0.0};
    metadata.sliceDirectionPatient = {0.0, 0.0, 1.0};
    metadata.rescaleSlope = 1.0;
    metadata.rescaleIntercept = -1024.0;
    metadata.minHu = -1024;
    metadata.maxHu = 3071;
    return metadata;
}

TEST(CoreCoordinateTests, MakeVolumeTransform_RoundTripsVoxelAndPatient)
{
    const VolumeMetadata metadata = makeValidMetadata();

    const auto transform = makeVolumeTransform(metadata);
    ASSERT_TRUE(transform.ok()) << transform.error().message;

    const Vec3d voxel{4.0, 5.0, 6.0};
    const Vec3d patient = voxelToPatient(transform.value(), voxel);
    EXPECT_TRUE(nearlyEqual(patient, Vec3d{12.0, 23.75, 37.5}));
    EXPECT_TRUE(nearlyEqual(patientToVoxel(transform.value(), patient), voxel));
}

TEST(CoreCoordinateTests, MakeVolumeTransform_RejectsInvalidSpacing)
{
    VolumeMetadata metadata = makeValidMetadata();
    metadata.spacingMm = {0.0, 0.75, 1.25};

    const auto transform = makeVolumeTransform(metadata);
    EXPECT_FALSE(transform.ok());
    EXPECT_EQ(transform.error().code, std::string(kErrorVolumeInvalidMetadata));
}

TEST(CoreCoordinateTests, ValidateVolumeMetadata_AcceptsCtMetadata)
{
    const auto validation = validateVolumeMetadata(makeValidMetadata());
    EXPECT_TRUE(validation.ok()) << validation.error().detail;
}

TEST(CoreCoordinateTests, ValidateVolumeMetadata_RejectsNonOrthogonalDirections)
{
    VolumeMetadata metadata = makeValidMetadata();
    metadata.columnDirectionPatient = {1.0, 0.0, 0.0};

    const auto validation = validateVolumeMetadata(metadata);

    ASSERT_FALSE(validation.ok());
    EXPECT_EQ(validation.error().code, std::string(kErrorVolumeInvalidMetadata));
    EXPECT_NE(validation.error().detail.find("orthogonal"), std::string::npos);
}

TEST(CoreCoordinateTests, ValidateVolumeMetadata_RejectsInvalidRescaleParameters)
{
    VolumeMetadata metadata = makeValidMetadata();
    metadata.rescaleSlope = 0.0;

    const auto validation = validateVolumeMetadata(metadata);

    ASSERT_FALSE(validation.ok());
    EXPECT_EQ(validation.error().code, std::string(kErrorVolumeInvalidMetadata));
    EXPECT_NE(validation.error().detail.find("rescaleSlope"), std::string::npos);
}

TEST(CoreCoordinateTests, MakeDenseHuVolume_SamplesVoxelHuInSliceMajorOrder)
{
    std::vector<int16_t> voxels{
        -1000, -900, -800,
        -700, -600, -500,
        10, 20, 30,
        40, 50, 60,
    };

    const auto image = makeDenseHuVolume({3, 2, 2}, voxels);

    ASSERT_TRUE(image.ok()) << image.error().detail;
    EXPECT_EQ(image.value()->dimensions().x, 3);
    EXPECT_EQ(image.value()->dimensions().y, 2);
    EXPECT_EQ(image.value()->dimensions().z, 2);
    EXPECT_EQ(image.value()->voxelHu(0, 0, 0), -1000);
    EXPECT_EQ(image.value()->voxelHu(2, 1, 0), -500);
    EXPECT_EQ(image.value()->voxelHu(2, 1, 1), 60);
    EXPECT_EQ(image.value()->voxels().size(), voxels.size());
}

TEST(CoreCoordinateTests, MakeDenseHuVolume_RejectsVoxelCountMismatch)
{
    const auto image = makeDenseHuVolume({3, 2, 2}, {1, 2, 3});

    ASSERT_FALSE(image.ok());
    EXPECT_EQ(image.error().code, std::string(kErrorVolumeImageSizeMismatch));
    EXPECT_NE(image.error().detail.find("expected=12"), std::string::npos);
}

TEST(CoreCoordinateTests, ErrorCodes_ExposeStableDicomImportCodes)
{
    EXPECT_EQ(std::string(kErrorDicomFolderNotFound), "DICOM_FOLDER_NOT_FOUND");
    EXPECT_EQ(std::string(kErrorDicomDependencyMissing), "DICOM_DEPENDENCY_MISSING");
    EXPECT_EQ(std::string(kErrorDicomEmptyFolder), "DICOM_EMPTY_FOLDER");
    EXPECT_EQ(std::string(kErrorDicomNoCtSeries), "DICOM_NO_CT_SERIES");
    EXPECT_EQ(std::string(kErrorDicomMultiSeriesUnsupported), "DICOM_MULTI_SERIES_UNSUPPORTED");
    EXPECT_EQ(std::string(kErrorDicomMissingTag), "DICOM_MISSING_TAG");
    EXPECT_EQ(std::string(kErrorDicomInconsistentGeometry), "DICOM_INCONSISTENT_GEOMETRY");
    EXPECT_EQ(std::string(kErrorDicomImageBuildFailed), "DICOM_IMAGE_BUILD_FAILED");
}

}  // namespace
}  // namespace measurement
