#include "InstrumentRenderModel.h"

#include <gtest/gtest.h>

namespace {

measurement::Instrument makeInstrument(double lengthMm)
{
    measurement::Instrument instrument;
    instrument.id = "pin-1";
    instrument.type = measurement::InstrumentType::GuidePin;
    instrument.entryPointPatientMm = {10.0, 20.0, 30.0};
    instrument.directionPatientUnit = {0.0, 1.0, 0.0};
    instrument.lengthMm = lengthMm;
    instrument.diameterMm = 2.0;
    instrument.visible = true;
    return instrument;
}

void expectNearVec(measurement::Vec3d actual, measurement::Vec3d expected)
{
    EXPECT_NEAR(actual.x, expected.x, 1.0e-6);
    EXPECT_NEAR(actual.y, expected.y, 1.0e-6);
    EXPECT_NEAR(actual.z, expected.z, 1.0e-6);
}

}  // namespace

TEST(InstrumentRenderModelTests, HeadLengthUsesProportionWithClamp)
{
    EXPECT_NEAR(measurement_app::InstrumentRenderModelBuilder::headLengthMm(70.0), 10.5, 1.0e-6);
    EXPECT_NEAR(measurement_app::InstrumentRenderModelBuilder::headLengthMm(20.0), 6.0, 1.0e-6);
    EXPECT_NEAR(measurement_app::InstrumentRenderModelBuilder::headLengthMm(10.0), 5.0, 1.0e-6);
    EXPECT_NEAR(measurement_app::InstrumentRenderModelBuilder::headLengthMm(200.0), 20.0, 1.0e-6);
}

TEST(InstrumentRenderModelTests, SegmentsReconnectToOriginalInstrument)
{
    measurement_app::InstrumentRenderModelBuilder builder;
    const measurement::Instrument instrument = makeInstrument(80.0);
    const auto model = builder.buildModel(instrument, "pin-1");

    EXPECT_TRUE(model.selected);
    EXPECT_EQ(model.headSegment.role, measurement_app::InstrumentRenderSegmentRole::Head);
    EXPECT_EQ(model.tailSegment.role, measurement_app::InstrumentRenderSegmentRole::Tail);
    EXPECT_NEAR(model.headSegment.lengthMm, 12.0, 1.0e-6);
    EXPECT_NEAR(model.tailSegment.lengthMm, 68.0, 1.0e-6);
    expectNearVec(model.headSegment.startPatientMm, instrument.entryPointPatientMm);
    expectNearVec(model.headSegment.endPatientMm, model.tailSegment.startPatientMm);
    expectNearVec(model.tailSegment.endPatientMm, measurement::endpointPatientMm(instrument));
}

TEST(InstrumentRenderModelTests, HiddenInstrumentDoesNotProduceModel)
{
    measurement::SurgicalPlan plan;
    measurement::Instrument hidden = makeInstrument(60.0);
    hidden.visible = false;
    ASSERT_TRUE(plan.addInstrument(hidden).ok());

    measurement_app::InstrumentRenderModelBuilder builder;
    EXPECT_TRUE(builder.buildVisibleModels(plan, hidden.id).empty());
    EXPECT_TRUE(builder.buildVisibleMeshSegments(plan, hidden.id).empty());
}

TEST(InstrumentRenderModelTests, LockedAndSelectedStatePropagatesToSegments)
{
    measurement::SurgicalPlan plan;
    measurement::Instrument instrument = makeInstrument(60.0);
    instrument.locked = true;
    ASSERT_TRUE(plan.addInstrument(instrument).ok());

    measurement_app::InstrumentRenderModelBuilder builder;
    const auto models = builder.buildVisibleModels(plan, instrument.id);
    ASSERT_EQ(models.size(), 1U);

    EXPECT_TRUE(models[0].selected);
    EXPECT_TRUE(models[0].locked);
    EXPECT_TRUE(models[0].headSegment.selected);
    EXPECT_TRUE(models[0].tailSegment.locked);
    EXPECT_LT(models[0].headSegment.style.opacity, 1.0);
}

TEST(InstrumentRenderModelTests, SectionSegmentsUseSplitGeometry)
{
    measurement::SurgicalPlan plan;
    const measurement::Instrument instrument = makeInstrument(80.0);
    ASSERT_TRUE(plan.addInstrument(instrument).ok());

    measurement_app::InstrumentRenderModelBuilder builder;
    const auto sections = builder.buildVisibleSectionSegments(
        plan,
        {{10.0, 20.0, 30.0}, {0.0, 0.0, 1.0}},
        instrument.id,
        16);

    EXPECT_FALSE(sections.empty());
    bool sawHead = false;
    bool sawTail = false;
    for (const auto& section : sections) {
        sawHead = sawHead || section.segment.role == measurement_app::InstrumentRenderSegmentRole::Head;
        sawTail = sawTail || section.segment.role == measurement_app::InstrumentRenderSegmentRole::Tail;
        EXPECT_EQ(section.segment.instrumentId, instrument.id);
        EXPECT_FALSE(section.segments.empty());
    }
    EXPECT_TRUE(sawHead);
    EXPECT_TRUE(sawTail);
}
