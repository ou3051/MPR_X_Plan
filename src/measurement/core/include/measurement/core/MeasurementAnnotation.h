#pragma once

#include "measurement/core/Geometry.h"

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace measurement {

class MeasurementId {
public:
    MeasurementId() = default;
    explicit MeasurementId(std::int64_t value);

    [[nodiscard]] std::int64_t value() const;
    [[nodiscard]] bool isValid() const;

    [[nodiscard]] bool operator==(MeasurementId other) const;
    [[nodiscard]] bool operator!=(MeasurementId other) const;
    [[nodiscard]] bool operator<(MeasurementId other) const;

private:
    std::int64_t m_value = -1;
};

enum class MeasurementType {
    Distance,
    Angle,
};

enum class MeasurementMode {
    Navigate,
    Distance,
    Angle,
};

enum class MeasurementVisibilityLevel {
    Hidden,
    SectionIndicator,
    FullDisplay,
};

enum class MeasurementViewType {
    Axial,
    Sagittal,
    Coronal,
    Oblique,
};

struct MeasurementPlane {
    Vec3d normal{0.0, 0.0, 1.0};
    Vec3d center{0.0, 0.0, 0.0};
    double thicknessMm = 1.0;

    [[nodiscard]] double signedDistance(Vec3d point) const;
    [[nodiscard]] std::optional<Vec3d> intersectSegment(Vec3d start, Vec3d end) const;
};

struct MeasurementAnnotation {
    MeasurementId id;
    MeasurementType type = MeasurementType::Distance;
    std::vector<Vec3d> worldPointsPatientMm;
    MeasurementPlane createdPlane;
    MeasurementViewType createdViewType = MeasurementViewType::Oblique;
    double value = 0.0;
    std::string label;
    std::string note;
    bool selected = false;

    [[nodiscard]] bool updateDistanceEndpoint(int pointIndex, Vec3d point);
    [[nodiscard]] bool updateAngleEndpoint(int pointIndex, Vec3d point);
    [[nodiscard]] bool recalculateValue();
    [[nodiscard]] std::string measurementText() const;
    [[nodiscard]] std::string displayText() const;
    [[nodiscard]] std::optional<Vec3d> anchorWorldPoint() const;

    [[nodiscard]] static MeasurementAnnotation makeDistance(Vec3d first, Vec3d second);
    [[nodiscard]] static MeasurementAnnotation makeAngle(Vec3d line1Start, Vec3d line1End, Vec3d line2Start, Vec3d line2End);
    [[nodiscard]] static std::optional<MeasurementAnnotation> tryMakeAngle(
        Vec3d line1Start,
        Vec3d line1End,
        Vec3d line2Start,
        Vec3d line2End);
};

struct MeasurementVisibilityResult {
    MeasurementId id;
    MeasurementType type = MeasurementType::Distance;
    MeasurementVisibilityLevel level = MeasurementVisibilityLevel::Hidden;
    std::vector<Vec3d> fullWorldPointsPatientMm;
    std::vector<Vec3d> sectionWorldPointsPatientMm;
    std::string label;
    std::string measurementText;
    std::string displayText;
    bool selected = false;
};

}  // namespace measurement

namespace std {

template <>
struct hash<measurement::MeasurementId> {
    [[nodiscard]] size_t operator()(measurement::MeasurementId id) const noexcept
    {
        return std::hash<std::int64_t>{}(id.value());
    }
};

}  // namespace std
