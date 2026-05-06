#include "measurement/planning/InstrumentGeometry.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace measurement {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kPlaneToleranceMm = 1.0e-6;

[[nodiscard]] Vec3d choosePerpendicular(Vec3d direction)
{
    const Vec3d axis = std::abs(direction.x) < 0.9 ? Vec3d{1.0, 0.0, 0.0} : Vec3d{0.0, 1.0, 0.0};
    return normalize(cross(direction, axis));
}

[[nodiscard]] bool isFinite(Vec3d value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

[[nodiscard]] bool samePoint(Vec3d lhs, Vec3d rhs)
{
    return nearlyEqual(lhs, rhs, 1.0e-5);
}

void addUniquePoint(std::vector<Vec3d>& points, Vec3d point)
{
    if (!isFinite(point)) {
        return;
    }
    const auto it = std::find_if(points.begin(), points.end(), [&](Vec3d existing) {
        return samePoint(existing, point);
    });
    if (it == points.end()) {
        points.push_back(point);
    }
}

void addPlaneEdgeIntersection(
    std::vector<Vec3d>& points,
    Vec3d a,
    Vec3d b,
    double da,
    double db)
{
    const bool aOnPlane = std::abs(da) <= kPlaneToleranceMm;
    const bool bOnPlane = std::abs(db) <= kPlaneToleranceMm;
    if (aOnPlane) {
        addUniquePoint(points, a);
    }
    if (bOnPlane) {
        addUniquePoint(points, b);
    }
    if ((da < -kPlaneToleranceMm && db > kPlaneToleranceMm)
        || (da > kPlaneToleranceMm && db < -kPlaneToleranceMm)) {
        const double t = da / (da - db);
        addUniquePoint(points, a + (b - a) * t);
    }
}

[[nodiscard]] MeshData buildCylinder(const Instrument& instrument, int radialSegments)
{
    MeshData mesh;
    if (!validateInstrument(instrument).ok() || radialSegments < 3) {
        return mesh;
    }

    const Vec3d axis = normalize(instrument.directionPatientUnit);
    const Vec3d u = choosePerpendicular(axis);
    const Vec3d v = normalize(cross(axis, u));
    const double radius = instrument.diameterMm * 0.5;
    const Vec3d start = instrument.entryPointPatientMm;
    const Vec3d end = endpointPatientMm(instrument);
    const unsigned int startCenterIndex = static_cast<unsigned int>(radialSegments * 2);
    const unsigned int endCenterIndex = startCenterIndex + 1U;

    for (int i = 0; i < radialSegments; ++i) {
        const double angle = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(radialSegments);
        const Vec3d radial = u * std::cos(angle) * radius + v * std::sin(angle) * radius;
        mesh.vertices.push_back(start + radial);
        mesh.vertices.push_back(end + radial);
    }
    mesh.vertices.push_back(start);
    mesh.vertices.push_back(end);

    for (int i = 0; i < radialSegments; ++i) {
        const int next = (i + 1) % radialSegments;
        const unsigned int a = static_cast<unsigned int>(i * 2);
        const unsigned int b = static_cast<unsigned int>(i * 2 + 1);
        const unsigned int c = static_cast<unsigned int>(next * 2);
        const unsigned int d = static_cast<unsigned int>(next * 2 + 1);
        mesh.indices.insert(mesh.indices.end(), {a, b, c, c, b, d});
        mesh.indices.insert(mesh.indices.end(), {startCenterIndex, c, a});
        mesh.indices.insert(mesh.indices.end(), {endCenterIndex, b, d});
    }
    return mesh;
}

void addTrianglePlaneSection(
    InstrumentSection& section,
    const InstrumentSlicePlane& plane,
    Vec3d a,
    Vec3d b,
    Vec3d c)
{
    const Vec3d normal = normalize(plane.normalPatientUnit);
    const double da = dot(a - plane.originPatientMm, normal);
    const double db = dot(b - plane.originPatientMm, normal);
    const double dc = dot(c - plane.originPatientMm, normal);

    std::vector<Vec3d> points;
    addPlaneEdgeIntersection(points, a, b, da, db);
    addPlaneEdgeIntersection(points, b, c, db, dc);
    addPlaneEdgeIntersection(points, c, a, dc, da);

    if (points.size() < 2U || samePoint(points[0], points[1])) {
        return;
    }
    section.segments.push_back({points[0], points[1]});
}

}  // namespace

MeshData InstrumentGeometryBuilder::buildGuidePinMesh(const Instrument& instrument, int radialSegments) const
{
    return buildCylinder(instrument, radialSegments);
}

MeshData InstrumentGeometryBuilder::buildPedicleScrewMesh(const Instrument& instrument, int radialSegments) const
{
    return buildCylinder(instrument, radialSegments);
}

MeshData InstrumentGeometryBuilder::buildMesh(const Instrument& instrument, int radialSegments) const
{
    switch (instrument.type) {
    case InstrumentType::GuidePin:
        return buildGuidePinMesh(instrument, radialSegments);
    case InstrumentType::PedicleScrew:
        return buildPedicleScrewMesh(instrument, radialSegments);
    }
    return {};
}

std::vector<InstrumentMesh> InstrumentGeometryBuilder::buildVisibleMeshes(
    const SurgicalPlan& plan,
    int radialSegments) const
{
    std::vector<InstrumentMesh> meshes;
    for (const Instrument& instrument : plan.instruments()) {
        if (!instrument.visible) {
            continue;
        }
        meshes.push_back({instrument.id, instrument.type, buildMesh(instrument, radialSegments)});
    }
    return meshes;
}

InstrumentSection InstrumentGeometryBuilder::buildSection(
    const Instrument& instrument,
    const InstrumentSlicePlane& plane,
    int radialSegments) const
{
    InstrumentSection section;
    section.instrumentId = instrument.id;
    section.instrumentType = instrument.type;

    if (!validateInstrument(instrument).ok()
        || radialSegments < 3
        || !isFinite(plane.originPatientMm)
        || !isFinite(plane.normalPatientUnit)
        || length(plane.normalPatientUnit) <= kPlaneToleranceMm) {
        return section;
    }

    const MeshData mesh = buildMesh(instrument, radialSegments);
    for (size_t index = 0; index + 2U < mesh.indices.size(); index += 3U) {
        const unsigned int ia = mesh.indices[index];
        const unsigned int ib = mesh.indices[index + 1U];
        const unsigned int ic = mesh.indices[index + 2U];
        if (ia >= mesh.vertices.size() || ib >= mesh.vertices.size() || ic >= mesh.vertices.size()) {
            continue;
        }
        addTrianglePlaneSection(section, plane, mesh.vertices[ia], mesh.vertices[ib], mesh.vertices[ic]);
    }
    return section;
}

std::vector<InstrumentSection> InstrumentGeometryBuilder::buildVisibleSections(
    const SurgicalPlan& plan,
    const InstrumentSlicePlane& plane,
    int radialSegments) const
{
    std::vector<InstrumentSection> sections;
    for (const Instrument& instrument : plan.instruments()) {
        if (!instrument.visible) {
            continue;
        }
        InstrumentSection section = buildSection(instrument, plane, radialSegments);
        if (!section.segments.empty()) {
            sections.push_back(std::move(section));
        }
    }
    return sections;
}

InstrumentPlanController::InstrumentPlanController(SurgicalPlan& plan)
    : m_plan(plan)
{
}

Result<void> InstrumentPlanController::createGuidePin(
    std::string id,
    Vec3d entryPointPatientMm,
    Vec3d directionPatientUnit,
    double lengthMm,
    double diameterMm,
    std::string label)
{
    return createInstrument(
        InstrumentType::GuidePin,
        std::move(id),
        entryPointPatientMm,
        directionPatientUnit,
        lengthMm,
        diameterMm,
        std::move(label));
}

Result<void> InstrumentPlanController::createPedicleScrew(
    std::string id,
    Vec3d entryPointPatientMm,
    Vec3d directionPatientUnit,
    double lengthMm,
    double diameterMm,
    std::string label)
{
    return createInstrument(
        InstrumentType::PedicleScrew,
        std::move(id),
        entryPointPatientMm,
        directionPatientUnit,
        lengthMm,
        diameterMm,
        std::move(label));
}

Result<void> InstrumentPlanController::updateInstrument(const std::string& id, const InstrumentPatch& patch)
{
    return m_plan.updateInstrument(id, patch);
}

Result<void> InstrumentPlanController::setInstrumentVisible(const std::string& id, bool visible)
{
    Result<InstrumentPatch> patch = patchFromCurrent(id);
    if (!patch.ok()) {
        return Result<void>::failure(patch.error());
    }
    patch.value().visible = visible;
    return updateInstrument(id, patch.value());
}

Result<void> InstrumentPlanController::lockInstrument(const std::string& id)
{
    Result<InstrumentPatch> patch = patchFromCurrent(id);
    if (!patch.ok()) {
        return Result<void>::failure(patch.error());
    }
    patch.value().locked = true;
    return updateInstrument(id, patch.value());
}

Result<void> InstrumentPlanController::removeInstrument(const std::string& id)
{
    const Instrument* instrument = m_plan.findInstrument(id);
    if (instrument == nullptr) {
        return Result<void>::failure({"INSTRUMENT_NOT_FOUND", "Instrument was not found.", id, true});
    }
    if (instrument->locked) {
        return Result<void>::failure({"INSTRUMENT_LOCKED", "Locked instruments cannot be removed from the UI.", id, true});
    }
    return m_plan.removeInstrument(id);
}

Result<void> InstrumentPlanController::createInstrument(
    InstrumentType type,
    std::string id,
    Vec3d entryPointPatientMm,
    Vec3d directionPatientUnit,
    double lengthMm,
    double diameterMm,
    std::string label)
{
    Instrument instrument;
    instrument.id = std::move(id);
    instrument.type = type;
    instrument.entryPointPatientMm = entryPointPatientMm;
    instrument.directionPatientUnit = normalize(directionPatientUnit);
    instrument.lengthMm = lengthMm;
    instrument.diameterMm = diameterMm;
    instrument.label = std::move(label);
    return m_plan.addInstrument(std::move(instrument));
}

Result<InstrumentPatch> InstrumentPlanController::patchFromCurrent(const std::string& id) const
{
    const Instrument* instrument = m_plan.findInstrument(id);
    if (instrument == nullptr) {
        return Result<InstrumentPatch>::failure({"INSTRUMENT_NOT_FOUND", "Instrument was not found.", id, true});
    }

    InstrumentPatch patch;
    patch.entryPointPatientMm = instrument->entryPointPatientMm;
    patch.directionPatientUnit = instrument->directionPatientUnit;
    patch.lengthMm = instrument->lengthMm;
    patch.diameterMm = instrument->diameterMm;
    patch.visible = instrument->visible;
    patch.locked = instrument->locked;
    patch.label = instrument->label;
    return Result<InstrumentPatch>::success(std::move(patch));
}

InstrumentPlacementController::InstrumentPlacementController(SurgicalPlan& plan)
    : m_plan(plan)
{
}

void InstrumentPlacementController::setSelectedInstrumentId(std::string id)
{
    m_selectedInstrumentId = std::move(id);
}

const std::string& InstrumentPlacementController::selectedInstrumentId() const
{
    return m_selectedInstrumentId;
}

void InstrumentPlacementController::setEditMode(InstrumentPlacementEditMode mode)
{
    m_editMode = mode;
}

InstrumentPlacementEditMode InstrumentPlacementController::editMode() const
{
    return m_editMode;
}

Result<void> InstrumentPlacementController::createGuidePinAtCrosshair(
    std::string id,
    Vec3d crosshairPatientMm,
    Vec3d directionPatientUnit,
    double lengthMm,
    double diameterMm,
    std::string label)
{
    return createAtCrosshair(
        InstrumentType::GuidePin,
        std::move(id),
        crosshairPatientMm,
        directionPatientUnit,
        lengthMm,
        diameterMm,
        std::move(label));
}

Result<void> InstrumentPlacementController::createPedicleScrewAtCrosshair(
    std::string id,
    Vec3d crosshairPatientMm,
    Vec3d directionPatientUnit,
    double lengthMm,
    double diameterMm,
    std::string label)
{
    return createAtCrosshair(
        InstrumentType::PedicleScrew,
        std::move(id),
        crosshairPatientMm,
        directionPatientUnit,
        lengthMm,
        diameterMm,
        std::move(label));
}

Result<void> InstrumentPlacementController::onCrosshairChanged(Vec3d crosshairPatientMm)
{
    if (m_editMode != InstrumentPlacementEditMode::BindEntryToCrosshair || m_selectedInstrumentId.empty()) {
        return Result<void>::success();
    }

    const Instrument* instrument = m_plan.findInstrument(m_selectedInstrumentId);
    if (instrument == nullptr) {
        return Result<void>::failure({"INSTRUMENT_NOT_FOUND", "Instrument was not found.", m_selectedInstrumentId, true});
    }

    InstrumentPatch patch;
    patch.entryPointPatientMm = crosshairPatientMm;
    patch.directionPatientUnit = instrument->directionPatientUnit;
    patch.lengthMm = instrument->lengthMm;
    patch.diameterMm = instrument->diameterMm;
    patch.visible = instrument->visible;
    patch.locked = instrument->locked;
    patch.label = instrument->label;
    return m_plan.updateInstrument(m_selectedInstrumentId, patch);
}

Result<void> InstrumentPlacementController::createAtCrosshair(
    InstrumentType type,
    std::string id,
    Vec3d crosshairPatientMm,
    Vec3d directionPatientUnit,
    double lengthMm,
    double diameterMm,
    std::string label)
{
    Instrument instrument;
    instrument.id = std::move(id);
    instrument.type = type;
    instrument.entryPointPatientMm = crosshairPatientMm;
    instrument.directionPatientUnit = normalize(directionPatientUnit);
    instrument.lengthMm = lengthMm;
    instrument.diameterMm = diameterMm;
    instrument.label = std::move(label);
    return m_plan.addInstrument(std::move(instrument));
}

}  // namespace measurement
