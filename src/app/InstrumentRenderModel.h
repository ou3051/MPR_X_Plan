#pragma once

#include "measurement/core/Instrument.h"
#include "measurement/planning/InstrumentGeometry.h"

#include <array>
#include <string>
#include <vector>

namespace measurement_app {

enum class InstrumentRenderSegmentRole {
    Head,
    Tail
};

struct InstrumentRenderStyle {
    std::array<double, 3> color{0.18, 0.82, 1.0};
    double opacity = 0.9;
    double lineWidth = 1.5;
};

struct InstrumentRenderSegment {
    std::string instrumentId;
    measurement::InstrumentType instrumentType = measurement::InstrumentType::GuidePin;
    InstrumentRenderSegmentRole role = InstrumentRenderSegmentRole::Head;
    measurement::Vec3d startPatientMm{};
    measurement::Vec3d endPatientMm{};
    measurement::Vec3d directionPatientUnit{0.0, 0.0, 1.0};
    double lengthMm = 0.0;
    double diameterMm = 1.0;
    bool selected = false;
    bool locked = false;
    InstrumentRenderStyle style;
};

struct InstrumentRenderModel {
    std::string instrumentId;
    measurement::InstrumentType instrumentType = measurement::InstrumentType::GuidePin;
    bool selected = false;
    bool locked = false;
    bool visible = true;
    InstrumentRenderSegment headSegment;
    InstrumentRenderSegment tailSegment;
};

struct InstrumentRenderMeshSegment {
    InstrumentRenderSegment segment;
    measurement::MeshData mesh;
};

struct InstrumentRenderSection {
    InstrumentRenderSegment segment;
    std::vector<measurement::InstrumentSectionSegment> segments;
};

class InstrumentRenderModelBuilder {
public:
    [[nodiscard]] InstrumentRenderModel buildModel(
        const measurement::Instrument& instrument,
        const std::string& selectedInstrumentId) const;

    [[nodiscard]] std::vector<InstrumentRenderModel> buildVisibleModels(
        const measurement::SurgicalPlan& plan,
        const std::string& selectedInstrumentId) const;

    [[nodiscard]] std::vector<InstrumentRenderMeshSegment> buildVisibleMeshSegments(
        const measurement::SurgicalPlan& plan,
        const std::string& selectedInstrumentId,
        int radialSegments = 24) const;

    [[nodiscard]] std::vector<InstrumentRenderSection> buildVisibleSectionSegments(
        const measurement::SurgicalPlan& plan,
        const measurement::InstrumentSlicePlane& plane,
        const std::string& selectedInstrumentId,
        int radialSegments = 48) const;

    [[nodiscard]] static double headLengthMm(double instrumentLengthMm);
    [[nodiscard]] static InstrumentRenderStyle styleFor(
        measurement::InstrumentType type,
        bool selected,
        bool locked,
        InstrumentRenderSegmentRole role);

private:
    [[nodiscard]] measurement::Instrument segmentInstrument(const InstrumentRenderSegment& segment) const;
};

}  // namespace measurement_app
