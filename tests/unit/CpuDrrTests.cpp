#include "measurement/drr/CpuDrrEngine.h"
#include "measurement/drr/CudaDrrEngine.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace measurement {
namespace {

class ConstantVolume final : public IImageVolume {
public:
    ConstantVolume(Size3i dimensions, int16_t hu)
        : m_dimensions(dimensions)
        , m_hu(hu)
    {
    }

    Size3i dimensions() const override { return m_dimensions; }
    int16_t voxelHu(int, int, int) const override { return m_hu; }

private:
    Size3i m_dimensions{};
    int16_t m_hu = 0;
};

class EmptyDimensionsVolume final : public IImageVolume {
public:
    Size3i dimensions() const override { return {0, 3, 3}; }
    int16_t voxelHu(int, int, int) const override { return 0; }
};

class DenseTestVolume final : public IImageVolume {
public:
    DenseTestVolume(Size3i dimensions, std::vector<int16_t> voxels)
        : m_dimensions(dimensions)
        , m_voxels(std::move(voxels))
    {
    }

    Size3i dimensions() const override { return m_dimensions; }

    int16_t voxelHu(int i, int j, int k) const override
    {
        const size_t sliceSize = static_cast<size_t>(m_dimensions.x) * static_cast<size_t>(m_dimensions.y);
        const size_t index = static_cast<size_t>(k) * sliceSize
            + static_cast<size_t>(j) * static_cast<size_t>(m_dimensions.x)
            + static_cast<size_t>(i);
        return m_voxels.at(index);
    }

private:
    Size3i m_dimensions{};
    std::vector<int16_t> m_voxels;
};

[[nodiscard]] VolumeMetadata makeMetadata(Size3i dimensions, Vec3d originPatientMm)
{
    VolumeMetadata metadata;
    metadata.dimensions = dimensions;
    metadata.spacingMm = {1.0, 1.0, 1.0};
    metadata.originPatientMm = originPatientMm;
    metadata.rowDirectionPatient = {1.0, 0.0, 0.0};
    metadata.columnDirectionPatient = {0.0, 1.0, 0.0};
    metadata.sliceDirectionPatient = {0.0, 0.0, 1.0};
    return metadata;
}

[[nodiscard]] VolumeData makeVolume(VolumeMetadata metadata, std::shared_ptr<IImageVolume> image)
{
    VolumeData volume;
    volume.metadata = metadata;
    volume.transform = makeVolumeTransform(metadata).value();
    volume.image = std::move(image);
    return volume;
}

[[nodiscard]] VolumeData makeConstantVolume(Size3i dimensions, int16_t hu)
{
    return makeVolume(makeMetadata(dimensions, {0.0, 0.0, 0.0}), std::make_shared<ConstantVolume>(dimensions, hu));
}

[[nodiscard]] VolumeData makeCenteredDenseVolume(Size3i dimensions, std::vector<int16_t> voxels)
{
    return makeVolume(
        makeMetadata(
            dimensions,
            {
                -static_cast<double>(dimensions.x - 1) * 0.5,
                -static_cast<double>(dimensions.y - 1) * 0.5,
                -static_cast<double>(dimensions.z - 1) * 0.5,
            }),
        std::make_shared<DenseTestVolume>(dimensions, std::move(voxels)));
}

[[nodiscard]] VolumeData makeCenteredSpherePhantom()
{
    const Size3i dimensions{9, 9, 9};
    std::vector<int16_t> voxels;
    voxels.reserve(static_cast<size_t>(dimensions.x * dimensions.y * dimensions.z));
    for (int k = 0; k < dimensions.z; ++k) {
        const double z = static_cast<double>(k) - 4.0;
        for (int j = 0; j < dimensions.y; ++j) {
            const double y = static_cast<double>(j) - 4.0;
            for (int i = 0; i < dimensions.x; ++i) {
                const double x = static_cast<double>(i) - 4.0;
                const double radius = std::sqrt(x * x + y * y + z * z);
                voxels.push_back(radius <= 2.5 ? 0 : -1000);
            }
        }
    }
    return makeCenteredDenseVolume(dimensions, std::move(voxels));
}

[[nodiscard]] VolumeData makeCenteredCylinderPhantom()
{
    const Size3i dimensions{9, 9, 9};
    std::vector<int16_t> voxels;
    voxels.reserve(static_cast<size_t>(dimensions.x * dimensions.y * dimensions.z));
    for (int k = 0; k < dimensions.z; ++k) {
        const double z = static_cast<double>(k) - 4.0;
        for (int j = 0; j < dimensions.y; ++j) {
            const double y = static_cast<double>(j) - 4.0;
            for (int i = 0; i < dimensions.x; ++i) {
                (void)i;
                const double radius = std::sqrt(y * y + z * z);
                voxels.push_back(radius <= 2.5 ? 0 : -1000);
            }
        }
    }
    return makeCenteredDenseVolume(dimensions, std::move(voxels));
}

[[nodiscard]] VolumeData makeStepHuPhantom()
{
    const Size3i dimensions{9, 9, 9};
    std::vector<int16_t> voxels;
    voxels.reserve(static_cast<size_t>(dimensions.x * dimensions.y * dimensions.z));
    for (int k = 0; k < dimensions.z; ++k) {
        (void)k;
        for (int j = 0; j < dimensions.y; ++j) {
            const double y = static_cast<double>(j) - 4.0;
            for (int i = 0; i < dimensions.x; ++i) {
                (void)i;
                voxels.push_back(y < 0.0 ? 0 : 1000);
            }
        }
    }
    return makeCenteredDenseVolume(dimensions, std::move(voxels));
}

[[nodiscard]] ProjectionParams makeCenteredProjection()
{
    ProjectionParams params;
    params.sourcePosPatientMm = {-12.0, 0.0, 0.0};
    params.detectorCenterPatientMm = {12.0, 0.0, 0.0};
    params.detectorUPatientUnit = {0.0, 1.0, 0.0};
    params.detectorVPatientUnit = {0.0, 0.0, 1.0};
    params.pixelSpacingMm = 1.0;
    return params;
}

[[nodiscard]] DrrRenderSettings makeSettings(int width, int height)
{
    DrrRenderSettings settings;
    settings.width = width;
    settings.height = height;
    settings.stepMm = 1.0;
    settings.windowCenter = 4.0;
    settings.windowWidth = 8.0;
    settings.gamma = 1.0;
    return settings;
}

[[nodiscard]] VolumeData makeTestVolume()
{
    return makeConstantVolume({3, 3, 3}, 0);
}

[[nodiscard]] float lineIntegralAt(const DrrImage& image, int x, int y)
{
    return image.lineIntegral.at(static_cast<size_t>(y * image.width + x));
}

[[nodiscard]] bool isExpectedCudaRuntimeError(const ErrorInfo& error)
{
    return error.code == "CUDA_DRR_DISABLED"
        || error.code == "CUDA_DRR_NO_DEVICE"
        || error.code == "CUDA_DRR_RUNTIME_ERROR";
}

}  // namespace

TEST(CpuDrrTests, SetVolume_RejectsEmptyVolume)
{
    CpuDrrEngine engine;

    const auto result = engine.setVolume({});

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, "DRR_VOLUME_EMPTY");
}

TEST(CpuDrrTests, SetVolume_RejectsInvalidDimensions)
{
    CpuDrrEngine engine;

    auto volume = makeTestVolume();
    volume.image = std::make_shared<EmptyDimensionsVolume>();

    const auto result = engine.setVolume(volume);

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, "DRR_VOLUME_EMPTY");
}

TEST(CpuDrrTests, Render_RejectsMissingVolume)
{
    CpuDrrEngine engine;

    const auto image = engine.render(makeCenteredProjection(), makeSettings(1, 1));

    ASSERT_FALSE(image.ok());
    EXPECT_EQ(image.error().code, "DRR_VOLUME_EMPTY");
}

TEST(CpuDrrTests, Render_RejectsInvalidSettings)
{
    CpuDrrEngine engine;
    ASSERT_TRUE(engine.setVolume(makeTestVolume()).ok());

    DrrRenderSettings settings = makeSettings(1, 1);
    settings.windowWidth = 0.0;

    const auto image = engine.render(makeCenteredProjection(), settings);

    ASSERT_FALSE(image.ok());
    EXPECT_EQ(image.error().code, "DRR_INVALID_SETTINGS");
}

TEST(CpuDrrTests, Render_RejectsInvalidProjection)
{
    CpuDrrEngine engine;
    ASSERT_TRUE(engine.setVolume(makeTestVolume()).ok());

    ProjectionParams params = makeCenteredProjection();
    params.detectorVPatientUnit = {0.0, 2.0, 0.0};

    const auto image = engine.render(params, makeSettings(1, 1));

    ASSERT_FALSE(image.ok());
    EXPECT_EQ(image.error().code, "DRR_INVALID_PROJECTION");
}

TEST(CpuDrrTests, Render_ProducesLineIntegralForSimpleVolume)
{
    CpuDrrEngine engine;
    ASSERT_TRUE(engine.setVolume(makeTestVolume()).ok());

    ProjectionParams params;
    params.sourcePosPatientMm = {-2.0, 1.0, 1.0};
    params.detectorCenterPatientMm = {4.0, 1.0, 1.0};
    params.detectorUPatientUnit = {0.0, 1.0, 0.0};
    params.detectorVPatientUnit = {0.0, 0.0, 1.0};
    params.pixelSpacingMm = 1.0;

    DrrRenderSettings settings = makeSettings(1, 1);
    settings.windowCenter = 1.0;
    settings.windowWidth = 4.0;

    const auto image = engine.render(params, settings);
    ASSERT_TRUE(image.ok()) << image.error().message;
    ASSERT_EQ(image.value().lineIntegral.size(), 1U);
    EXPECT_NEAR(image.value().lineIntegral[0], 3.0F, 1.0e-5F);
}

TEST(CpuDrrTests, Render_AppliesHuScaleAndOffsetInAttenuationFormula)
{
    CpuDrrEngine engine;
    ASSERT_TRUE(engine.setVolume(makeTestVolume()).ok());

    ProjectionParams params;
    params.sourcePosPatientMm = {-2.0, 1.0, 1.0};
    params.detectorCenterPatientMm = {4.0, 1.0, 1.0};
    params.detectorUPatientUnit = {0.0, 1.0, 0.0};
    params.detectorVPatientUnit = {0.0, 0.0, 1.0};
    params.pixelSpacingMm = 1.0;

    DrrRenderSettings settings = makeSettings(1, 1);
    settings.huScale = 2.0;
    settings.huOffset = 500.0;

    const auto image = engine.render(params, settings);
    ASSERT_TRUE(image.ok()) << image.error().message;
    ASSERT_EQ(image.value().lineIntegral.size(), 1U);
    EXPECT_NEAR(image.value().lineIntegral[0], 4.5F, 1.0e-5F);
}

TEST(CpuDrrTests, Render_ProducesDeterministicOutput)
{
    CpuDrrEngine engine;
    ASSERT_TRUE(engine.setVolume(makeCenteredSpherePhantom()).ok());

    const auto first = engine.render(makeCenteredProjection(), makeSettings(7, 3));
    const auto second = engine.render(makeCenteredProjection(), makeSettings(7, 3));

    ASSERT_TRUE(first.ok()) << first.error().message;
    ASSERT_TRUE(second.ok()) << second.error().message;
    EXPECT_EQ(first.value().lineIntegral, second.value().lineIntegral);
    EXPECT_EQ(first.value().displayImage, second.value().displayImage);
}

TEST(CpuDrrTests, Render_SpherePhantomCenterExceedsEdge)
{
    CpuDrrEngine engine;
    ASSERT_TRUE(engine.setVolume(makeCenteredSpherePhantom()).ok());

    const auto image = engine.render(makeCenteredProjection(), makeSettings(15, 1));

    ASSERT_TRUE(image.ok()) << image.error().message;
    ASSERT_EQ(image.value().lineIntegral.size(), 15U);
    EXPECT_GT(lineIntegralAt(image.value(), 7, 0), lineIntegralAt(image.value(), 0, 0));
    EXPECT_GT(lineIntegralAt(image.value(), 7, 0), 0.0F);
    EXPECT_NEAR(lineIntegralAt(image.value(), 0, 0), 0.0F, 1.0e-5F);
}

TEST(CpuDrrTests, Render_CylinderPhantomCenterExceedsEdge)
{
    CpuDrrEngine engine;
    ASSERT_TRUE(engine.setVolume(makeCenteredCylinderPhantom()).ok());

    const auto image = engine.render(makeCenteredProjection(), makeSettings(15, 1));

    ASSERT_TRUE(image.ok()) << image.error().message;
    EXPECT_GT(lineIntegralAt(image.value(), 7, 0), lineIntegralAt(image.value(), 0, 0));
    EXPECT_GT(lineIntegralAt(image.value(), 7, 0), 0.0F);
}

TEST(CpuDrrTests, Render_StepHuPhantomDifferentiatesDetectorColumns)
{
    CpuDrrEngine engine;
    ASSERT_TRUE(engine.setVolume(makeStepHuPhantom()).ok());

    const auto image = engine.render(makeCenteredProjection(), makeSettings(3, 1));

    ASSERT_TRUE(image.ok()) << image.error().message;
    ASSERT_EQ(image.value().lineIntegral.size(), 3U);
    EXPECT_GT(lineIntegralAt(image.value(), 2, 0), lineIntegralAt(image.value(), 0, 0));
}

TEST(CpuDrrTests, Render_MapsDisplayWithWindowAndGamma)
{
    CpuDrrEngine engine;
    ASSERT_TRUE(engine.setVolume(makeTestVolume()).ok());

    ProjectionParams params;
    params.sourcePosPatientMm = {-2.0, 1.0, 1.0};
    params.detectorCenterPatientMm = {4.0, 1.0, 1.0};
    params.detectorUPatientUnit = {0.0, 1.0, 0.0};
    params.detectorVPatientUnit = {0.0, 0.0, 1.0};
    params.pixelSpacingMm = 1.0;

    DrrRenderSettings linearSettings = makeSettings(1, 1);
    linearSettings.windowCenter = 3.0;
    linearSettings.windowWidth = 6.0;
    linearSettings.gamma = 1.0;

    DrrRenderSettings gammaSettings = linearSettings;
    gammaSettings.gamma = 2.0;

    const auto linearImage = engine.render(params, linearSettings);
    const auto gammaImage = engine.render(params, gammaSettings);

    ASSERT_TRUE(linearImage.ok()) << linearImage.error().message;
    ASSERT_TRUE(gammaImage.ok()) << gammaImage.error().message;
    ASSERT_EQ(linearImage.value().displayImage.size(), 1U);
    ASSERT_EQ(gammaImage.value().displayImage.size(), 1U);
    EXPECT_GT(linearImage.value().displayImage[0], 0U);
    EXPECT_LT(linearImage.value().displayImage[0], 65535U);
    EXPECT_GT(gammaImage.value().displayImage[0], linearImage.value().displayImage[0]);
}

TEST(CpuDrrTests, Render_CanOmitLineIntegralButKeepsDisplayImage)
{
    CpuDrrEngine engine;
    ASSERT_TRUE(engine.setVolume(makeTestVolume()).ok());

    DrrRenderSettings settings = makeSettings(1, 1);
    settings.outputLineIntegral = false;

    ProjectionParams params;
    params.sourcePosPatientMm = {-2.0, 1.0, 1.0};
    params.detectorCenterPatientMm = {4.0, 1.0, 1.0};
    params.detectorUPatientUnit = {0.0, 1.0, 0.0};
    params.detectorVPatientUnit = {0.0, 0.0, 1.0};
    params.pixelSpacingMm = 1.0;

    const auto image = engine.render(params, settings);

    ASSERT_TRUE(image.ok()) << image.error().message;
    EXPECT_TRUE(image.value().lineIntegral.empty());
    EXPECT_EQ(image.value().displayImage.size(), 1U);
}

TEST(CpuDrrTests, CudaRender_MatchesCpuOrReportsClearRuntimeError)
{
    const VolumeData volume = makeCenteredSpherePhantom();
    const ProjectionParams projection = makeCenteredProjection();
    const DrrRenderSettings settings = makeSettings(3, 3);

    CpuDrrEngine cpuEngine;
    ASSERT_TRUE(cpuEngine.setVolume(volume).ok());
    const auto cpuImage = cpuEngine.render(projection, settings);
    ASSERT_TRUE(cpuImage.ok()) << cpuImage.error().message;

    CudaDrrEngine cudaEngine;
    ASSERT_TRUE(cudaEngine.setVolume(volume).ok());
    const auto cudaImage = cudaEngine.render(projection, settings);

    if (!cudaImage.ok()) {
        EXPECT_TRUE(isExpectedCudaRuntimeError(cudaImage.error()))
            << cudaImage.error().code << ": " << cudaImage.error().message;
        return;
    }

    ASSERT_EQ(cudaImage.value().lineIntegral.size(), cpuImage.value().lineIntegral.size());
    ASSERT_EQ(cudaImage.value().displayImage.size(), cpuImage.value().displayImage.size());
    for (size_t index = 0; index < cpuImage.value().lineIntegral.size(); ++index) {
        EXPECT_NEAR(cudaImage.value().lineIntegral[index], cpuImage.value().lineIntegral[index], 1.0e-4F);
    }
}

TEST(CpuDrrTests, CudaRender_ReusesUploadedVolumeForRepeatedRenders)
{
    const VolumeData volume = makeCenteredCylinderPhantom();
    const ProjectionParams projection = makeCenteredProjection();
    const DrrRenderSettings settings = makeSettings(5, 5);

    CudaDrrEngine cudaEngine;
    const auto volumeResult = cudaEngine.setVolume(volume);
    if (!volumeResult.ok()) {
        EXPECT_TRUE(isExpectedCudaRuntimeError(volumeResult.error()))
            << volumeResult.error().code << ": " << volumeResult.error().message;
        return;
    }

    const auto first = cudaEngine.render(projection, settings);
    const auto second = cudaEngine.render(projection, settings);
    if (!first.ok() || !second.ok()) {
        const ErrorInfo& error = first.ok() ? second.error() : first.error();
        EXPECT_TRUE(isExpectedCudaRuntimeError(error)) << error.code << ": " << error.message;
        return;
    }

    EXPECT_EQ(first.value().lineIntegral, second.value().lineIntegral);
    EXPECT_EQ(first.value().displayImage, second.value().displayImage);
}

TEST(CpuDrrTests, CudaRender_ReuploadsWhenVolumeChanges)
{
    const ProjectionParams projection = makeCenteredProjection();
    const DrrRenderSettings settings = makeSettings(3, 3);

    CudaDrrEngine cudaEngine;
    const auto firstVolume = cudaEngine.setVolume(makeCenteredSpherePhantom());
    if (!firstVolume.ok()) {
        EXPECT_TRUE(isExpectedCudaRuntimeError(firstVolume.error()))
            << firstVolume.error().code << ": " << firstVolume.error().message;
        return;
    }

    const auto sphereImage = cudaEngine.render(projection, settings);
    if (!sphereImage.ok()) {
        EXPECT_TRUE(isExpectedCudaRuntimeError(sphereImage.error()))
            << sphereImage.error().code << ": " << sphereImage.error().message;
        return;
    }

    const auto secondVolume = cudaEngine.setVolume(makeStepHuPhantom());
    if (!secondVolume.ok()) {
        EXPECT_TRUE(isExpectedCudaRuntimeError(secondVolume.error()))
            << secondVolume.error().code << ": " << secondVolume.error().message;
        return;
    }

    const auto stepImage = cudaEngine.render(projection, settings);
    if (!stepImage.ok()) {
        EXPECT_TRUE(isExpectedCudaRuntimeError(stepImage.error()))
            << stepImage.error().code << ": " << stepImage.error().message;
        return;
    }
    EXPECT_NE(sphereImage.value().lineIntegral, stepImage.value().lineIntegral);
}

TEST(CpuDrrTests, CudaRender_CanOmitLineIntegralButKeepsDisplayImage)
{
    CudaDrrEngine cudaEngine;
    const auto volumeResult = cudaEngine.setVolume(makeCenteredSpherePhantom());
    if (!volumeResult.ok()) {
        EXPECT_TRUE(isExpectedCudaRuntimeError(volumeResult.error()))
            << volumeResult.error().code << ": " << volumeResult.error().message;
        return;
    }

    DrrRenderSettings settings = makeSettings(3, 3);
    settings.outputLineIntegral = false;
    const auto image = cudaEngine.render(makeCenteredProjection(), settings);
    if (!image.ok()) {
        EXPECT_TRUE(isExpectedCudaRuntimeError(image.error()))
            << image.error().code << ": " << image.error().message;
        return;
    }

    EXPECT_TRUE(image.value().lineIntegral.empty());
    EXPECT_EQ(image.value().displayImage.size(), 9U);
}
}  // namespace measurement
