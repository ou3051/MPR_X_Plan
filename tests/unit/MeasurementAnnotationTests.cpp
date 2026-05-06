#include "measurement/core/MeasurementAnnotation.h"
#include "measurement/core/MeasurementStateMachine.h"
#include "measurement/core/MeasurementStore.h"
#include "measurement/core/MeasurementVisibility.h"

#include <gtest/gtest.h>

namespace measurement {
namespace {

class CountingMeasurementObserver final : public IMeasurementStoreObserver {
public:
    void onMeasurementAdded(MeasurementId) override { ++added; }
    void onMeasurementRemoved(MeasurementId) override { ++removed; }
    void onMeasurementChanged(MeasurementId id) override
    {
        ++changed;
        lastChanged = id;
    }
    void onMeasurementsCleared() override { ++cleared; }

    int added = 0;
    int removed = 0;
    int changed = 0;
    int cleared = 0;
    MeasurementId lastChanged;
};

[[nodiscard]] MeasurementPlane axialSlice(double z = 0.0, double thicknessMm = 2.0)
{
    return {{0.0, 0.0, 1.0}, {0.0, 0.0, z}, thicknessMm};
}

}  // namespace

TEST(MeasurementAnnotationTests, DistanceAndAngle_FormatStableMeasurements)
{
    const MeasurementAnnotation distance = MeasurementAnnotation::makeDistance({0.0, 0.0, 0.0}, {3.0, 4.0, 12.0});
    EXPECT_EQ(distance.type, MeasurementType::Distance);
    EXPECT_DOUBLE_EQ(distance.value, 13.0);
    EXPECT_EQ(distance.measurementText(), "13.0 mm");

    const auto angle = MeasurementAnnotation::tryMakeAngle(
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0});
    ASSERT_TRUE(angle.has_value());
    EXPECT_EQ(angle->type, MeasurementType::Angle);
    EXPECT_DOUBLE_EQ(angle->value, 90.0);
    EXPECT_EQ(angle->measurementText(), "90.0 deg");
}

TEST(MeasurementAnnotationTests, AngleEndpointUpdate_RollsBackInvalidEdits)
{
    MeasurementAnnotation angle = MeasurementAnnotation::makeAngle(
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0});

    EXPECT_FALSE(angle.updateAngleEndpoint(1, {0.0, 0.0, 0.0}));
    ASSERT_EQ(angle.worldPointsPatientMm.size(), 4U);
    EXPECT_TRUE(nearlyEqual(angle.worldPointsPatientMm[1], Vec3d{1.0, 0.0, 0.0}));
    EXPECT_DOUBLE_EQ(angle.value, 90.0);
}

TEST(MeasurementStoreTests, NotifiesObserversAndTracksSelectionMetadata)
{
    MeasurementStore store;
    CountingMeasurementObserver observer;
    store.addObserver(&observer);

    const MeasurementId distanceId = store.add(MeasurementAnnotation::makeDistance({0.0, 0.0, 0.0}, {0.0, 6.0, 8.0}));
    EXPECT_TRUE(distanceId.isValid());
    EXPECT_EQ(observer.added, 1);

    EXPECT_TRUE(store.updateDistanceEndpoint(distanceId, 1, {0.0, 0.0, 12.0}));
    EXPECT_EQ(observer.changed, 1);
    EXPECT_EQ(observer.lastChanged, distanceId);
    ASSERT_TRUE(store.find(distanceId).has_value());
    EXPECT_EQ(store.find(distanceId)->measurementText(), "12.0 mm");

    EXPECT_TRUE(store.rename(distanceId, "Canal width"));
    EXPECT_EQ(store.find(distanceId)->displayText(), "Canal width");

    const auto anchor = store.anchorWorldPoint(distanceId);
    ASSERT_TRUE(anchor.has_value());
    EXPECT_TRUE(nearlyEqual(*anchor, Vec3d{0.0, 0.0, 6.0}));

    EXPECT_TRUE(store.remove(distanceId));
    EXPECT_EQ(observer.removed, 1);
    EXPECT_FALSE(store.find(distanceId).has_value());
}

TEST(MeasurementStateMachineTests, CompletesDistanceAndAngleAfterRequiredClicks)
{
    MeasurementStateMachine stateMachine;
    MeasurementAnnotation completed;

    stateMachine.setMode(MeasurementMode::Distance);
    EXPECT_FALSE(stateMachine.addPoint({0.0, 0.0, 0.0}, completed));
    EXPECT_TRUE(stateMachine.addPoint({3.0, 4.0, 0.0}, completed));
    EXPECT_EQ(completed.type, MeasurementType::Distance);
    EXPECT_DOUBLE_EQ(completed.value, 5.0);

    stateMachine.setMode(MeasurementMode::Angle);
    EXPECT_FALSE(stateMachine.addPoint({0.0, 0.0, 0.0}, completed));
    EXPECT_FALSE(stateMachine.addPoint({1.0, 0.0, 0.0}, completed));
    EXPECT_FALSE(stateMachine.addPoint({0.0, 0.0, 0.0}, completed));
    EXPECT_TRUE(stateMachine.addPoint({0.0, 1.0, 0.0}, completed));
    EXPECT_EQ(completed.type, MeasurementType::Angle);
    EXPECT_DOUBLE_EQ(completed.value, 90.0);
}

TEST(MeasurementVisibilityTests, EvaluatesFullAndSectionDisplayForCurrentSlice)
{
    MeasurementAnnotation distance = MeasurementAnnotation::makeDistance({0.0, 0.0, 0.0}, {10.0, 0.0, 0.0});
    distance.id = MeasurementId(7);

    auto result = measurement_visibility::evaluate(distance, axialSlice(), MeasurementViewType::Axial);
    EXPECT_EQ(result.id, MeasurementId(7));
    EXPECT_EQ(result.level, MeasurementVisibilityLevel::FullDisplay);
    EXPECT_EQ(result.fullWorldPointsPatientMm.size(), 2U);

    distance = MeasurementAnnotation::makeDistance({0.0, 0.0, -5.0}, {0.0, 0.0, 5.0});
    result = measurement_visibility::evaluate(distance, axialSlice(), MeasurementViewType::Axial);
    EXPECT_EQ(result.level, MeasurementVisibilityLevel::SectionIndicator);
    EXPECT_EQ(result.sectionWorldPointsPatientMm.size(), 2U);

    distance = MeasurementAnnotation::makeDistance({0.0, 0.0, 5.0}, {10.0, 0.0, 5.0});
    result = measurement_visibility::evaluate(distance, axialSlice(), MeasurementViewType::Axial);
    EXPECT_EQ(result.level, MeasurementVisibilityLevel::Hidden);
}

}  // namespace measurement
