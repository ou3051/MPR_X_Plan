#include "measurement/mpr/MprResliceEngine.h"
#include "measurement/vtk/VtkMprResliceAdapter.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <utility>
#include <vector>

namespace measurement {
namespace {

[[nodiscard]] VolumeData makeTestVolume(Size3i dimensions = {5, 6, 7})
{
    VolumeMetadata metadata;
    metadata.dimensions = dimensions;
    metadata.spacingMm = {2.0, 3.0, 4.0};
    metadata.originPatientMm = {10.0, 20.0, 30.0};
    metadata.rowDirectionPatient = {1.0, 0.0, 0.0};
    metadata.columnDirectionPatient = {0.0, 1.0, 0.0};
    metadata.sliceDirectionPatient = {0.0, 0.0, 1.0};
    metadata.minHu = -1024;
    metadata.maxHu = 2048;

    std::vector<int16_t> voxels;
    voxels.reserve(static_cast<size_t>(dimensions.x) * static_cast<size_t>(dimensions.y) * static_cast<size_t>(dimensions.z));
    for (int k = 0; k < dimensions.z; ++k) {
        for (int j = 0; j < dimensions.y; ++j) {
            for (int i = 0; i < dimensions.x; ++i) {
                voxels.push_back(static_cast<int16_t>(i + j * 10 + k * 100));
            }
        }
    }

    VolumeData volume;
    volume.metadata = metadata;
    volume.transform = makeVolumeTransform(metadata).value();
    volume.image = makeDenseHuVolume(dimensions, std::move(voxels)).value();
    return volume;
}

[[nodiscard]] Vec3d centerPatient(const VolumeData& volume)
{
    return voxelToPatient(volume.transform, {
        static_cast<double>(volume.metadata.dimensions.x - 1) * 0.5,
        static_cast<double>(volume.metadata.dimensions.y - 1) * 0.5,
        static_cast<double>(volume.metadata.dimensions.z - 1) * 0.5,
    });
}

[[nodiscard]] MprSliceRequest smallRequest()
{
    return {16, 12, 1.0};
}

}  // namespace

TEST(MprResliceTests, Mpr_DefaultPlaneFrames_ReturnExpectedNormals)
{
    const VolumeData volume = makeTestVolume();
    const Vec3d origin = centerPatient(volume);

    const auto axial = defaultSliceFrame(volume.metadata, MprPlane::Axial, origin);
    ASSERT_TRUE(axial.ok()) << axial.error().detail;
    EXPECT_TRUE(nearlyEqual(axial.value().normalPatientUnit, Vec3d{0.0, 0.0, 1.0}));
    EXPECT_TRUE(nearlyEqual(cross(axial.value().horizontalPatientUnit, axial.value().verticalPatientUnit), axial.value().normalPatientUnit));

    const auto sagittal = defaultSliceFrame(volume.metadata, MprPlane::Sagittal, origin);
    ASSERT_TRUE(sagittal.ok()) << sagittal.error().detail;
    EXPECT_TRUE(nearlyEqual(sagittal.value().normalPatientUnit, Vec3d{1.0, 0.0, 0.0}));
    EXPECT_TRUE(nearlyEqual(cross(sagittal.value().horizontalPatientUnit, sagittal.value().verticalPatientUnit), sagittal.value().normalPatientUnit));

    const auto coronal = defaultSliceFrame(volume.metadata, MprPlane::Coronal, origin);
    ASSERT_TRUE(coronal.ok()) << coronal.error().detail;
    EXPECT_TRUE(nearlyEqual(coronal.value().normalPatientUnit, Vec3d{0.0, 1.0, 0.0}));
    EXPECT_TRUE(nearlyEqual(cross(coronal.value().horizontalPatientUnit, coronal.value().verticalPatientUnit), coronal.value().normalPatientUnit));
}

TEST(MprResliceTests, Mpr_ObliqueFrame_UsesExternalAxes)
{
    const VolumeData volume = makeTestVolume();
    MprViewState state;
    state.crosshairPatientMm = centerPatient(volume);
    state.obliqueFrame = MprSliceFrame{
        state.crosshairPatientMm,
        normalize(Vec3d{1.0, 1.0, 0.0}),
        {0.0, 0.0, 1.0},
        normalize(Vec3d{1.0, -1.0, 0.0}),
    };

    const auto parameters = buildMprResliceParameters(volume, state, smallRequest());
    ASSERT_TRUE(parameters.ok()) << parameters.error().detail;
    EXPECT_TRUE(nearlyEqual(parameters.value().frame.originPatientMm, state.crosshairPatientMm));
    EXPECT_TRUE(nearlyEqual(parameters.value().frame.horizontalPatientUnit, state.obliqueFrame->horizontalPatientUnit));
    EXPECT_TRUE(nearlyEqual(parameters.value().frame.verticalPatientUnit, state.obliqueFrame->verticalPatientUnit));
    EXPECT_TRUE(nearlyEqual(cross(parameters.value().frame.horizontalPatientUnit, parameters.value().frame.verticalPatientUnit), parameters.value().frame.normalPatientUnit));
}

TEST(MprResliceTests, Mpr_InvalidInputs_ReturnRecoverableErrors)
{
    VolumeData emptyVolume;
    MprViewState state;
    state.crosshairPatientMm = {0.0, 0.0, 0.0};
    auto parameters = buildMprResliceParameters(emptyVolume, state, smallRequest());
    ASSERT_FALSE(parameters.ok());
    EXPECT_EQ(parameters.error().code, "MPR_VOLUME_EMPTY");
    EXPECT_TRUE(parameters.error().recoverable);

    VolumeData volume = makeTestVolume();
    state.crosshairPatientMm = {1000.0, 1000.0, 1000.0};
    parameters = buildMprResliceParameters(volume, state, smallRequest());
    ASSERT_FALSE(parameters.ok());
    EXPECT_EQ(parameters.error().code, "MPR_CROSSHAIR_OUT_OF_BOUNDS");

    state.crosshairPatientMm = centerPatient(volume);
    state.zoom = 0.0;
    parameters = buildMprResliceParameters(volume, state, smallRequest());
    ASSERT_FALSE(parameters.ok());
    EXPECT_EQ(parameters.error().code, "MPR_FRAME_INVALID");

    state.zoom = 1.0;
    state.obliqueFrame = MprSliceFrame{
        state.crosshairPatientMm,
        {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0},
    };
    parameters = buildMprResliceParameters(volume, state, smallRequest());
    ASSERT_FALSE(parameters.ok());
    EXPECT_EQ(parameters.error().code, "MPR_FRAME_INVALID");

    state.obliqueFrame.reset();
    parameters = buildMprResliceParameters(volume, state, {0, 12, 1.0});
    ASSERT_FALSE(parameters.ok());
    EXPECT_EQ(parameters.error().code, "MPR_OUTPUT_INVALID");
}

TEST(MprResliceTests, Mpr_ZoomAndPan_AdjustOutputSpacingAndOrigin)
{
    const VolumeData volume = makeTestVolume();
    MprViewState state;
    state.crosshairPatientMm = centerPatient(volume);
    state.zoom = 2.0;
    state.pan = {4.0, 6.0, 0.0};

    const auto parameters = buildMprResliceParameters(volume, state, smallRequest());
    ASSERT_TRUE(parameters.ok()) << parameters.error().detail;
    EXPECT_DOUBLE_EQ(parameters.value().request.pixelSpacingMm, 0.5);
    EXPECT_TRUE(nearlyEqual(parameters.value().frame.originPatientMm, state.crosshairPatientMm + Vec3d{4.0, 6.0, 0.0}));
}

TEST(MprResliceTests, Mpr_VtkAdapter_ProducesRenderableImageWhenVtkAvailable)
{
    const VolumeData volume = makeTestVolume();
    MprViewState state;
    state.crosshairPatientMm = centerPatient(volume);
    state.obliqueFrame = MprSliceFrame{
        state.crosshairPatientMm,
        normalize(Vec3d{1.0, 1.0, 0.0}),
        {0.0, 0.0, 1.0},
        normalize(Vec3d{1.0, -1.0, 0.0}),
    };

    VtkMprResliceAdapter adapter;
    const auto result = adapter.reslice(volume, state, smallRequest());

#if MEASUREMENT_HAVE_VTK
    ASSERT_TRUE(result.ok()) << result.error().detail;
    EXPECT_TRUE(result.value().readyToRender);
    EXPECT_TRUE(result.value().imageChanged);
    ASSERT_NE(result.value().image, nullptr);
    EXPECT_EQ(result.value().width, smallRequest().outputWidth);
    EXPECT_EQ(result.value().height, smallRequest().outputHeight);
#else
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, "MPR_VTK_UNAVAILABLE");
#endif
}

}  // namespace measurement
