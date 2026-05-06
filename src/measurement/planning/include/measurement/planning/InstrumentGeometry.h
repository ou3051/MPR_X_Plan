#pragma once

#include "measurement/core/Geometry.h"
#include "measurement/core/Instrument.h"

#include <string>
#include <vector>

namespace measurement {

struct MeshData {
    std::vector<Vec3d> vertices;
    std::vector<unsigned int> indices;
};

struct InstrumentMesh {
    std::string instrumentId;
    InstrumentType instrumentType = InstrumentType::GuidePin;
    MeshData mesh;
};

struct InstrumentSlicePlane {
    Vec3d originPatientMm;
    Vec3d normalPatientUnit{0.0, 0.0, 1.0};
};

struct InstrumentSectionSegment {
    Vec3d startPatientMm;
    Vec3d endPatientMm;
};

struct InstrumentSection {
    std::string instrumentId;
    InstrumentType instrumentType = InstrumentType::GuidePin;
    std::vector<InstrumentSectionSegment> segments;
};

class InstrumentGeometryBuilder {
public:
    [[nodiscard]] MeshData buildGuidePinMesh(const Instrument& instrument, int radialSegments = 24) const;
    [[nodiscard]] MeshData buildPedicleScrewMesh(const Instrument& instrument, int radialSegments = 24) const;
    [[nodiscard]] MeshData buildMesh(const Instrument& instrument, int radialSegments = 24) const;
    [[nodiscard]] std::vector<InstrumentMesh> buildVisibleMeshes(
        const SurgicalPlan& plan,
        int radialSegments = 24) const;
    [[nodiscard]] InstrumentSection buildSection(
        const Instrument& instrument,
        const InstrumentSlicePlane& plane,
        int radialSegments = 48) const;
    [[nodiscard]] std::vector<InstrumentSection> buildVisibleSections(
        const SurgicalPlan& plan,
        const InstrumentSlicePlane& plane,
        int radialSegments = 48) const;
};

class InstrumentPlanController {
public:
    explicit InstrumentPlanController(SurgicalPlan& plan);

    [[nodiscard]] Result<void> createGuidePin(
        std::string id,
        Vec3d entryPointPatientMm,
        Vec3d directionPatientUnit,
        double lengthMm,
        double diameterMm,
        std::string label = {});
    [[nodiscard]] Result<void> createPedicleScrew(
        std::string id,
        Vec3d entryPointPatientMm,
        Vec3d directionPatientUnit,
        double lengthMm,
        double diameterMm,
        std::string label = {});
    [[nodiscard]] Result<void> updateInstrument(const std::string& id, const InstrumentPatch& patch);
    [[nodiscard]] Result<void> setInstrumentVisible(const std::string& id, bool visible);
    [[nodiscard]] Result<void> lockInstrument(const std::string& id);
    [[nodiscard]] Result<void> removeInstrument(const std::string& id);

private:
    [[nodiscard]] Result<void> createInstrument(
        InstrumentType type,
        std::string id,
        Vec3d entryPointPatientMm,
        Vec3d directionPatientUnit,
        double lengthMm,
        double diameterMm,
        std::string label);
    [[nodiscard]] Result<InstrumentPatch> patchFromCurrent(const std::string& id) const;

    SurgicalPlan& m_plan;
};

enum class InstrumentPlacementEditMode {
    None,
    BindEntryToCrosshair
};

class InstrumentPlacementController {
public:
    explicit InstrumentPlacementController(SurgicalPlan& plan);

    void setSelectedInstrumentId(std::string id);
    [[nodiscard]] const std::string& selectedInstrumentId() const;
    void setEditMode(InstrumentPlacementEditMode mode);
    [[nodiscard]] InstrumentPlacementEditMode editMode() const;

    [[nodiscard]] Result<void> createGuidePinAtCrosshair(
        std::string id,
        Vec3d crosshairPatientMm,
        Vec3d directionPatientUnit,
        double lengthMm,
        double diameterMm,
        std::string label = {});
    [[nodiscard]] Result<void> createPedicleScrewAtCrosshair(
        std::string id,
        Vec3d crosshairPatientMm,
        Vec3d directionPatientUnit,
        double lengthMm,
        double diameterMm,
        std::string label = {});
    [[nodiscard]] Result<void> onCrosshairChanged(Vec3d crosshairPatientMm);

private:
    [[nodiscard]] Result<void> createAtCrosshair(
        InstrumentType type,
        std::string id,
        Vec3d crosshairPatientMm,
        Vec3d directionPatientUnit,
        double lengthMm,
        double diameterMm,
        std::string label);

    SurgicalPlan& m_plan;
    std::string m_selectedInstrumentId;
    InstrumentPlacementEditMode m_editMode = InstrumentPlacementEditMode::None;
};

}  // namespace measurement
