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

TEST(InstrumentTests, GeometryBuilder_CreatesCylinderMesh)
{
    InstrumentGeometryBuilder builder;
    const MeshData mesh = builder.buildGuidePinMesh(validInstrument(), 8);
    EXPECT_EQ(mesh.vertices.size(), 16U);
    EXPECT_EQ(mesh.indices.size(), 48U);
}

}  // namespace
}  // namespace measurement
