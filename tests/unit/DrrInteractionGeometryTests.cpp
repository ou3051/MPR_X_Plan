#include "DrrInteractionGeometry.h"

#include <gtest/gtest.h>

#include <cmath>

namespace {

measurement::ProjectionParams makeApProjection()
{
    measurement::ProjectionParams projection;
    projection.sourcePosPatientMm = {0.0, -700.0, 0.0};
    projection.detectorCenterPatientMm = {0.0, 300.0, 0.0};
    projection.detectorUPatientUnit = {1.0, 0.0, 0.0};
    projection.detectorVPatientUnit = {0.0, 0.0, 1.0};
    projection.pixelSpacingMm = 1.0;
    projection.detectorWidth = 320;
    projection.detectorHeight = 240;
    projection.sidMm = 1000.0;
    projection.sodMm = 700.0;
    return projection;
}

measurement::ProjectionParams makeLatProjection()
{
    measurement::ProjectionParams projection;
    projection.sourcePosPatientMm = {-700.0, 0.0, 0.0};
    projection.detectorCenterPatientMm = {300.0, 0.0, 0.0};
    projection.detectorUPatientUnit = {0.0, 1.0, 0.0};
    projection.detectorVPatientUnit = {0.0, 0.0, 1.0};
    projection.pixelSpacingMm = 1.0;
    projection.detectorWidth = 320;
    projection.detectorHeight = 240;
    projection.sidMm = 1000.0;
    projection.sodMm = 700.0;
    return projection;
}

void expectNearVec(measurement::Vec3d actual, measurement::Vec3d expected, double tolerance)
{
    EXPECT_NEAR(actual.x, expected.x, tolerance);
    EXPECT_NEAR(actual.y, expected.y, tolerance);
    EXPECT_NEAR(actual.z, expected.z, tolerance);
}

}  // namespace

TEST(DrrInteractionGeometryTests, ProjectAndRayRoundTripDetectorPixel)
{
    const measurement::ProjectionParams projection = makeApProjection();
    const measurement_app::DrrDetectorPoint pixel{174.0, 132.0};

    const auto ray = measurement_app::detectorPixelToPatientRay(projection, pixel);
    ASSERT_TRUE(ray.has_value());

    const measurement::Vec3d detectorPoint = projection.sourcePosPatientMm + ray->directionPatientUnit * 1000.0;
    const auto projected = measurement_app::projectPatientToDetectorPixel(projection, detectorPoint);
    ASSERT_TRUE(projected.has_value());
    EXPECT_NEAR(projected->x, pixel.x, 1.0e-6);
    EXPECT_NEAR(projected->y, pixel.y, 1.0e-6);
}

TEST(DrrInteractionGeometryTests, ApLatRaysReconstructPatientPoint)
{
    const measurement::ProjectionParams ap = makeApProjection();
    const measurement::ProjectionParams lat = makeLatProjection();
    const measurement::Vec3d point{14.0, -22.0, 31.0};

    const auto apPixel = measurement_app::projectPatientToDetectorPixel(ap, point);
    const auto latPixel = measurement_app::projectPatientToDetectorPixel(lat, point);
    ASSERT_TRUE(apPixel.has_value());
    ASSERT_TRUE(latPixel.has_value());

    const auto apRay = measurement_app::detectorPixelToPatientRay(ap, *apPixel);
    const auto latRay = measurement_app::detectorPixelToPatientRay(lat, *latPixel);
    ASSERT_TRUE(apRay.has_value());
    ASSERT_TRUE(latRay.has_value());

    const auto reconstructed = measurement_app::closestPointBetweenRays(*apRay, *latRay);
    ASSERT_TRUE(reconstructed.has_value());
    EXPECT_LT(reconstructed->distanceMm, 1.0e-6);
    expectNearVec(reconstructed->pointPatientMm, point, 1.0e-5);
}

TEST(DrrInteractionGeometryTests, SkewRayPairReportsDistance)
{
    const auto reconstructed = measurement_app::closestPointBetweenRays(
        {{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}},
        {{50.0, 0.0, 50.0}, {0.0, 1.0, 0.1}});
    ASSERT_TRUE(reconstructed.has_value());
    EXPECT_GT(reconstructed->distanceMm, 25.0);
}

TEST(DrrInteractionGeometryTests, HeadDragPlaneIntersectionKeepsDepthPlane)
{
    const measurement::ProjectionParams projection = makeApProjection();
    const measurement::Vec3d head{12.0, 18.0, 24.0};
    const measurement_app::DrrDetectorPoint targetPixel{168.0, 135.0};
    const auto ray = measurement_app::detectorPixelToPatientRay(projection, targetPixel);
    ASSERT_TRUE(ray.has_value());

    const measurement::Vec3d detectorNormal = measurement::normalize(
        measurement::cross(projection.detectorUPatientUnit, projection.detectorVPatientUnit));
    const auto movedHead = measurement_app::rayPlaneIntersection(*ray, head, detectorNormal);
    ASSERT_TRUE(movedHead.has_value());

    EXPECT_NEAR(measurement::dot(*movedHead - head, detectorNormal), 0.0, 1.0e-6);
    const auto projected = measurement_app::projectPatientToDetectorPixel(projection, *movedHead);
    ASSERT_TRUE(projected.has_value());
    EXPECT_NEAR(projected->x, targetPixel.x, 1.0e-6);
    EXPECT_NEAR(projected->y, targetPixel.y, 1.0e-6);
}

TEST(DrrInteractionGeometryTests, TailDragSphereIntersectionKeepsHeadAndLength)
{
    const measurement::ProjectionParams projection = makeApProjection();
    const measurement::Vec3d head{0.0, 0.0, 0.0};
    const double lengthMm = 60.0;
    const measurement::Vec3d desiredTail{20.0, 30.0, std::sqrt(lengthMm * lengthMm - 20.0 * 20.0 - 30.0 * 30.0)};
    const auto targetPixel = measurement_app::projectPatientToDetectorPixel(projection, desiredTail);
    ASSERT_TRUE(targetPixel.has_value());
    const auto ray = measurement_app::detectorPixelToPatientRay(projection, *targetPixel);
    ASSERT_TRUE(ray.has_value());

    const auto movedTail = measurement_app::raySphereIntersectionNearDirection(
        *ray,
        head,
        lengthMm,
        measurement::normalize(desiredTail - head));
    ASSERT_TRUE(movedTail.has_value());

    EXPECT_NEAR(measurement::length(*movedTail - head), lengthMm, 1.0e-5);
    const auto projected = measurement_app::projectPatientToDetectorPixel(projection, *movedTail);
    ASSERT_TRUE(projected.has_value());
    EXPECT_NEAR(projected->x, targetPixel->x, 1.0e-5);
    EXPECT_NEAR(projected->y, targetPixel->y, 1.0e-5);
}
