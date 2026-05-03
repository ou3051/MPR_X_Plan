#include "measurement/drr/CpuDrrEngine.h"

#include <gtest/gtest.h>

namespace measurement {
namespace {

class ConstantVolume final : public IImageVolume {
public:
    explicit ConstantVolume(int16_t hu)
        : m_hu(hu)
    {
    }

    Size3i dimensions() const override { return {3, 3, 3}; }
    int16_t voxelHu(int, int, int) const override { return m_hu; }

private:
    int16_t m_hu = 0;
};

[[nodiscard]] VolumeData makeTestVolume()
{
    VolumeMetadata metadata;
    metadata.dimensions = {3, 3, 3};
    metadata.spacingMm = {1.0, 1.0, 1.0};
    metadata.originPatientMm = {0.0, 0.0, 0.0};
    metadata.rowDirectionPatient = {1.0, 0.0, 0.0};
    metadata.columnDirectionPatient = {0.0, 1.0, 0.0};
    metadata.sliceDirectionPatient = {0.0, 0.0, 1.0};

    VolumeData volume;
    volume.metadata = metadata;
    volume.transform = makeVolumeTransform(metadata).value();
    volume.image = std::make_shared<ConstantVolume>(0);
    return volume;
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

    DrrRenderSettings settings;
    settings.width = 1;
    settings.height = 1;
    settings.stepMm = 1.0;
    settings.windowCenter = 1.0;
    settings.windowWidth = 4.0;

    const auto image = engine.render(params, settings);
    ASSERT_TRUE(image.ok()) << image.error().message;
    ASSERT_EQ(image.value().lineIntegral.size(), 1U);
    EXPECT_GT(image.value().lineIntegral[0], 0.0F);
}

}  // namespace
}  // namespace measurement
