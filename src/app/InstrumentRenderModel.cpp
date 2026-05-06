#include "InstrumentRenderModel.h"

#include <algorithm>
#include <cmath>

namespace measurement_app {
namespace {

[[nodiscard]] measurement::Vec3d safeDirection(measurement::Vec3d direction)
{
    const measurement::Vec3d normalized = measurement::normalize(direction);
    if (!std::isfinite(normalized.x) || !std::isfinite(normalized.y) || !std::isfinite(normalized.z)
        || measurement::length(normalized) <= 1.0e-9) {
        return {0.0, 0.0, 1.0};
    }
    return normalized;
}

[[nodiscard]] std::array<double, 3> baseColor(measurement::InstrumentType type, bool selected, bool locked)
{
    if (locked) {
        return {0.70, 0.73, 0.78};
    }
    if (selected) {
        return {1.0, 0.84, 0.18};
    }
    if (type == measurement::InstrumentType::GuidePin) {
        return {0.18, 0.82, 1.0};
    }
    return {1.0, 0.32, 0.24};
}

}  // namespace

double InstrumentRenderModelBuilder::headLengthMm(double instrumentLengthMm)
{
    if (!std::isfinite(instrumentLengthMm) || instrumentLengthMm <= 0.0) {
        return 0.0;
    }
    const double proportional = std::clamp(instrumentLengthMm * 0.15, 6.0, 20.0);
    return std::min(proportional, instrumentLengthMm * 0.5);
}

InstrumentRenderStyle InstrumentRenderModelBuilder::styleFor(
    measurement::InstrumentType type,
    bool selected,
    bool locked,
    InstrumentRenderSegmentRole role)
{
    InstrumentRenderStyle style;
    style.color = baseColor(type, selected, locked);
    style.opacity = locked ? 0.66 : (selected ? 1.0 : 0.92);
    style.lineWidth = selected ? 2.5 : 1.5;
    if (role == InstrumentRenderSegmentRole::Head) {
        style.opacity = std::min(1.0, style.opacity + 0.06);
        style.lineWidth += selected ? 0.8 : 0.4;
    }
    return style;
}

InstrumentRenderModel InstrumentRenderModelBuilder::buildModel(
    const measurement::Instrument& instrument,
    const std::string& selectedInstrumentId) const
{
    InstrumentRenderModel model;
    model.instrumentId = instrument.id;
    model.instrumentType = instrument.type;
    model.selected = instrument.id == selectedInstrumentId;
    model.locked = instrument.locked;
    model.visible = instrument.visible;

    const measurement::Vec3d direction = safeDirection(instrument.directionPatientUnit);
    const double totalLength = std::max(0.0, instrument.lengthMm);
    const double headLength = headLengthMm(totalLength);
    const double tailLength = std::max(0.0, totalLength - headLength);
    const measurement::Vec3d headStart = instrument.entryPointPatientMm;
    const measurement::Vec3d splitPoint = headStart + direction * headLength;
    const measurement::Vec3d tailEnd = headStart + direction * totalLength;

    const auto makeSegment = [&](InstrumentRenderSegmentRole role, measurement::Vec3d start, measurement::Vec3d end, double length) {
        InstrumentRenderSegment segment;
        segment.instrumentId = instrument.id;
        segment.instrumentType = instrument.type;
        segment.role = role;
        segment.startPatientMm = start;
        segment.endPatientMm = end;
        segment.directionPatientUnit = direction;
        segment.lengthMm = length;
        segment.diameterMm = instrument.diameterMm;
        segment.selected = model.selected;
        segment.locked = instrument.locked;
        segment.style = styleFor(instrument.type, model.selected, instrument.locked, role);
        return segment;
    };

    model.headSegment = makeSegment(InstrumentRenderSegmentRole::Head, headStart, splitPoint, headLength);
    model.tailSegment = makeSegment(InstrumentRenderSegmentRole::Tail, splitPoint, tailEnd, tailLength);
    return model;
}

std::vector<InstrumentRenderModel> InstrumentRenderModelBuilder::buildVisibleModels(
    const measurement::SurgicalPlan& plan,
    const std::string& selectedInstrumentId) const
{
    std::vector<InstrumentRenderModel> models;
    for (const measurement::Instrument& instrument : plan.instruments()) {
        if (!instrument.visible) {
            continue;
        }
        models.push_back(buildModel(instrument, selectedInstrumentId));
    }
    return models;
}

std::vector<InstrumentRenderMeshSegment> InstrumentRenderModelBuilder::buildVisibleMeshSegments(
    const measurement::SurgicalPlan& plan,
    const std::string& selectedInstrumentId,
    int radialSegments) const
{
    std::vector<InstrumentRenderMeshSegment> renderMeshes;
    measurement::InstrumentGeometryBuilder geometryBuilder;
    for (const InstrumentRenderModel& model : buildVisibleModels(plan, selectedInstrumentId)) {
        for (const InstrumentRenderSegment* segment : {&model.headSegment, &model.tailSegment}) {
            if (segment->lengthMm <= 1.0e-6) {
                continue;
            }
            renderMeshes.push_back({*segment, geometryBuilder.buildMesh(segmentInstrument(*segment), radialSegments)});
        }
    }
    return renderMeshes;
}

std::vector<InstrumentRenderSection> InstrumentRenderModelBuilder::buildVisibleSectionSegments(
    const measurement::SurgicalPlan& plan,
    const measurement::InstrumentSlicePlane& plane,
    const std::string& selectedInstrumentId,
    int radialSegments) const
{
    std::vector<InstrumentRenderSection> renderSections;
    measurement::InstrumentGeometryBuilder geometryBuilder;
    for (const InstrumentRenderModel& model : buildVisibleModels(plan, selectedInstrumentId)) {
        for (const InstrumentRenderSegment* segment : {&model.headSegment, &model.tailSegment}) {
            if (segment->lengthMm <= 1.0e-6) {
                continue;
            }
            measurement::InstrumentSection section = geometryBuilder.buildSection(
                segmentInstrument(*segment),
                plane,
                radialSegments);
            if (!section.segments.empty()) {
                renderSections.push_back({*segment, std::move(section.segments)});
            }
        }
    }
    return renderSections;
}

measurement::Instrument InstrumentRenderModelBuilder::segmentInstrument(const InstrumentRenderSegment& segment) const
{
    measurement::Instrument instrument;
    instrument.id = segment.instrumentId;
    instrument.type = segment.instrumentType;
    instrument.entryPointPatientMm = segment.startPatientMm;
    instrument.directionPatientUnit = segment.directionPatientUnit;
    instrument.lengthMm = segment.lengthMm;
    instrument.diameterMm = segment.diameterMm;
    instrument.visible = true;
    instrument.locked = segment.locked;
    instrument.label = segment.instrumentId;
    return instrument;
}

}  // namespace measurement_app
