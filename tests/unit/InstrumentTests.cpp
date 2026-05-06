#include "measurement/core/Instrument.h"
#include "measurement/planning/InstrumentGeometry.h"

#include <gtest/gtest.h>

namespace measurement {
namespace {

[[nodiscard]] Instrument validInstrument()
{
    Instrument instrument;
    instrument.id = "pin-1";
    instrument.type = InstrumentType::GuidePin;
    instrument.entryPointPatientMm = {1.0, 2.0, 3.0};
    instrument.directionPatientUnit = {0.0, 0.0, 1.0};
    instrument.lengthMm = 50.0;
    instrument.diameterMm = 2.0;
    return instrument;
}

[[nodiscard]] Instrument pedicleScrew()
{
    Instrument instrument;
    instrument.id = "screw-1";
    instrument.type = InstrumentType::PedicleScrew;
    instrument.entryPointPatientMm = {0.0, 0.0, 0.0};
    instrument.directionPatientUnit = {0.0, 0.0, 1.0};
    instrument.lengthMm = 10.0;
    instrument.diameterMm = 4.0;
    return instrument;
}

void expectSectionOnPlane(const InstrumentSection& section, const InstrumentSlicePlane& plane)
{
    const Vec3d normal = normalize(plane.normalPatientUnit);
    ASSERT_FALSE(section.segments.empty());
    for (const InstrumentSectionSegment& segment : section.segments) {
        EXPECT_NEAR(dot(segment.startPatientMm - plane.originPatientMm, normal), 0.0, 1.0e-5);
        EXPECT_NEAR(dot(segment.endPatientMm - plane.originPatientMm, normal), 0.0, 1.0e-5);
    }
}

TEST(InstrumentTests, Endpoint_UsesEntryDirectionAndLength)
{
    const Instrument instrument = validInstrument();
    EXPECT_TRUE(nearlyEqual(endpointPatientMm(instrument), Vec3d{1.0, 2.0, 53.0}));
}

TEST(InstrumentTests, SurgicalPlan_AddsAndFindsInstrument)
{
    SurgicalPlan plan;
    const auto result = plan.addInstrument(validInstrument());
    ASSERT_TRUE(result.ok()) << result.error().message;
    ASSERT_NE(plan.findInstrument("pin-1"), nullptr);
}

TEST(InstrumentTests, Controller_UpdatesVisibilityAndPreventsLockedRemoval)
{
    SurgicalPlan plan;
    InstrumentPlanController controller(plan);

    ASSERT_TRUE(controller.createGuidePin("pin-1", {1.0, 2.0, 3.0}, {0.0, 0.0, 2.0}, 50.0, 2.0, "Guide").ok());
    ASSERT_TRUE(controller.setInstrumentVisible("pin-1", false).ok());
    ASSERT_FALSE(plan.findInstrument("pin-1")->visible);

    ASSERT_TRUE(controller.lockInstrument("pin-1").ok());
    const Result<void> removeResult = controller.removeInstrument("pin-1");
    ASSERT_FALSE(removeResult.ok());
    EXPECT_EQ(removeResult.error().code, "INSTRUMENT_LOCKED");
    ASSERT_NE(plan.findInstrument("pin-1"), nullptr);
}

TEST(InstrumentTests, GeometryBuilder_SkipsHiddenInstrumentsForDisplayMeshes)
{
    SurgicalPlan plan;
    InstrumentPlanController controller(plan);

    ASSERT_TRUE(controller.createGuidePin("visible", {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, 40.0, 2.0).ok());
    ASSERT_TRUE(controller.createPedicleScrew("hidden", {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 35.0, 5.5).ok());
    ASSERT_TRUE(controller.setInstrumentVisible("hidden", false).ok());

    InstrumentGeometryBuilder builder;
    const std::vector<InstrumentMesh> meshes = builder.buildVisibleMeshes(plan, 8);
    ASSERT_EQ(meshes.size(), 1U);
    EXPECT_EQ(meshes[0].instrumentId, "visible");
    EXPECT_EQ(meshes[0].mesh.vertices.size(), 18U);
}

TEST(InstrumentTests, GeometryBuilder_CreatesCylinderMesh)
{
    InstrumentGeometryBuilder builder;
    const MeshData mesh = builder.buildGuidePinMesh(validInstrument(), 8);
    EXPECT_EQ(mesh.vertices.size(), 18U);
    EXPECT_EQ(mesh.indices.size(), 96U);
}

TEST(InstrumentTests, PlacementController_CreatesInstrumentAtCrosshairWithViewNormal)
{
    SurgicalPlan plan;
    InstrumentPlacementController placement(plan);
    const Vec3d crosshair{10.0, 20.0, 30.0};
    const Vec3d viewNormal{0.0, 2.0, 0.0};

    const Result<void> result = placement.createPedicleScrewAtCrosshair("screw-1", crosshair, viewNormal, 45.0, 6.5, "Screw");
    ASSERT_TRUE(result.ok()) << result.error().message;

    const Instrument* instrument = plan.findInstrument("screw-1");
    ASSERT_NE(instrument, nullptr);
    EXPECT_TRUE(nearlyEqual(instrument->entryPointPatientMm, crosshair));
    EXPECT_TRUE(nearlyEqual(instrument->directionPatientUnit, Vec3d{0.0, 1.0, 0.0}));
}

TEST(InstrumentTests, PlacementController_CrosshairDoesNotMoveInstrumentWhenBindingDisabled)
{
    SurgicalPlan plan;
    InstrumentPlacementController placement(plan);
    ASSERT_TRUE(placement.createGuidePinAtCrosshair("pin-1", {1.0, 2.0, 3.0}, {0.0, 0.0, 1.0}, 50.0, 2.0).ok());
    placement.setSelectedInstrumentId("pin-1");
    placement.setEditMode(InstrumentPlacementEditMode::None);

    ASSERT_TRUE(placement.onCrosshairChanged({9.0, 8.0, 7.0}).ok());
    EXPECT_TRUE(nearlyEqual(plan.findInstrument("pin-1")->entryPointPatientMm, Vec3d{1.0, 2.0, 3.0}));
}

TEST(InstrumentTests, PlacementController_BindModeMovesEntryAndPreservesShape)
{
    SurgicalPlan plan;
    InstrumentPlacementController placement(plan);
    ASSERT_TRUE(placement.createPedicleScrewAtCrosshair("screw-1", {1.0, 2.0, 3.0}, {1.0, 0.0, 0.0}, 45.0, 6.5).ok());
    placement.setSelectedInstrumentId("screw-1");
    placement.setEditMode(InstrumentPlacementEditMode::BindEntryToCrosshair);

    ASSERT_TRUE(placement.onCrosshairChanged({4.0, 5.0, 6.0}).ok());
    const Instrument* instrument = plan.findInstrument("screw-1");
    ASSERT_NE(instrument, nullptr);
    EXPECT_TRUE(nearlyEqual(instrument->entryPointPatientMm, Vec3d{4.0, 5.0, 6.0}));
    EXPECT_TRUE(nearlyEqual(instrument->directionPatientUnit, Vec3d{1.0, 0.0, 0.0}));
    EXPECT_DOUBLE_EQ(instrument->lengthMm, 45.0);
    EXPECT_DOUBLE_EQ(instrument->diameterMm, 6.5);
}

TEST(InstrumentTests, PlacementController_BindModeDoesNotModifyLockedInstrument)
{
    SurgicalPlan plan;
    InstrumentPlacementController placement(plan);
    InstrumentPlanController planController(plan);
    ASSERT_TRUE(placement.createGuidePinAtCrosshair("pin-1", {1.0, 2.0, 3.0}, {0.0, 0.0, 1.0}, 50.0, 2.0).ok());
    ASSERT_TRUE(planController.lockInstrument("pin-1").ok());
    placement.setSelectedInstrumentId("pin-1");
    placement.setEditMode(InstrumentPlacementEditMode::BindEntryToCrosshair);

    const Result<void> result = placement.onCrosshairChanged({9.0, 8.0, 7.0});
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, "INSTRUMENT_LOCKED");
    EXPECT_TRUE(nearlyEqual(plan.findInstrument("pin-1")->entryPointPatientMm, Vec3d{1.0, 2.0, 3.0}));
}

TEST(InstrumentTests, GeometryBuilder_BuildsPedicleScrewSectionFrom3dBody)
{
    InstrumentGeometryBuilder builder;
    const Instrument screw = pedicleScrew();
    const InstrumentSlicePlane crossSectionPlane{{0.0, 0.0, 5.0}, {0.0, 0.0, 1.0}};

    const InstrumentSection section = builder.buildSection(screw, crossSectionPlane, 16);
    expectSectionOnPlane(section, crossSectionPlane);
    EXPECT_EQ(section.instrumentId, "screw-1");
    EXPECT_EQ(section.instrumentType, InstrumentType::PedicleScrew);
}

TEST(InstrumentTests, GeometryBuilder_BuildsLongitudinalSectionFrom3dBody)
{
    InstrumentGeometryBuilder builder;
    const Instrument screw = pedicleScrew();
    const InstrumentSlicePlane longitudinalPlane{{0.0, 0.0, 5.0}, {0.0, 1.0, 0.0}};

    const InstrumentSection section = builder.buildSection(screw, longitudinalPlane, 16);
    expectSectionOnPlane(section, longitudinalPlane);
}

TEST(InstrumentTests, GeometryBuilder_SectionChangesWithPlaneAndSkipsHiddenInstruments)
{
    SurgicalPlan plan;
    Instrument visible = pedicleScrew();
    visible.id = "visible";
    Instrument hidden = pedicleScrew();
    hidden.id = "hidden";
    hidden.visible = false;
    ASSERT_TRUE(plan.addInstrument(visible).ok());
    ASSERT_TRUE(plan.addInstrument(hidden).ok());

    InstrumentGeometryBuilder builder;
    const auto centerSections = builder.buildVisibleSections(plan, {{0.0, 0.0, 5.0}, {0.0, 0.0, 1.0}}, 16);
    ASSERT_EQ(centerSections.size(), 1U);
    EXPECT_EQ(centerSections[0].instrumentId, "visible");

    const auto outsideSections = builder.buildVisibleSections(plan, {{0.0, 0.0, 20.0}, {0.0, 0.0, 1.0}}, 16);
    EXPECT_TRUE(outsideSections.empty());
}

}  // namespace
}  // namespace measurement
