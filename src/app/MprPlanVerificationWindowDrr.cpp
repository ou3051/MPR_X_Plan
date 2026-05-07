#include "MprPlanVerificationWindow.h"

#include "DrrInteractionGeometry.h"
#include "DrrRenderRequest.h"
#include "MprSliceWidget.h"
#include "PlanSceneWidget.h"
#include "XrayDisplayWidget.h"

#include <QDoubleSpinBox>
#include <QPushButton>
#include <QStatusBar>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string>

namespace measurement_app {
namespace {

[[nodiscard]] bool isFiniteVec(measurement::Vec3d value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

[[nodiscard]] int xrayPresetIndex(measurement::XrayPreset preset)
{
    return preset == measurement::XrayPreset::AP ? 0 : 1;
}

}  // namespace

void MprPlanVerificationWindow::refreshXrayViews()
{
    const DrrUiSettings apSettings = drrSettingsFromControls(measurement::XrayPreset::AP);
    const DrrUiSettings latSettings = drrSettingsFromControls(measurement::XrayPreset::LAT);
    const auto apPlacementConstraints = drrPlacementConstraintsForView(measurement::XrayPreset::AP);
    const auto latPlacementConstraints = drrPlacementConstraintsForView(measurement::XrayPreset::LAT);
    const bool latPlacementReady = latPlacementConstraints[0].has_value() && latPlacementConstraints[1].has_value();
    std::array<measurement::ProjectionParams, 2> projections{};
    std::array<bool, 2> projectionEnabled{false, false};
    measurement::DrrRenderSettings unusedRenderSettings;
    projectionEnabled[0] = buildDrrRenderRequest(
        &m_volume,
        measurement::XrayPreset::AP,
        apSettings,
        projections[0],
        unusedRenderSettings);
    projectionEnabled[1] = buildDrrRenderRequest(
        &m_volume,
        measurement::XrayPreset::LAT,
        latSettings,
        projections[1],
        unusedRenderSettings);

    if (m_apXrayView != nullptr) {
        m_apXrayView->setVolume(&m_volume);
        m_apXrayView->setPlan(&m_plan);
        m_apXrayView->setSelectedInstrumentId(selectedInstrumentId());
        m_apXrayView->setDrrSettings(apSettings);
        m_apXrayView->setPlacementActive(m_drrPlacementType.has_value() && !m_pendingDrrLines[0].has_value());
        m_apXrayView->setPendingLine(m_pendingDrrLines[0]);
        m_apXrayView->setPlacementConstraints(apPlacementConstraints);
        m_apXrayView->refreshImage();
    }
    if (m_latXrayView != nullptr) {
        m_latXrayView->setVolume(&m_volume);
        m_latXrayView->setPlan(&m_plan);
        m_latXrayView->setSelectedInstrumentId(selectedInstrumentId());
        m_latXrayView->setDrrSettings(latSettings);
        m_latXrayView->setPlacementActive(
            m_drrPlacementType.has_value()
            && m_pendingDrrLines[0].has_value()
            && !m_pendingDrrLines[1].has_value()
            && latPlacementReady);
        m_latXrayView->setPendingLine(m_pendingDrrLines[1]);
        m_latXrayView->setPlacementConstraints(latPlacementConstraints);
        m_latXrayView->refreshImage();
    }

    if (m_sceneView != nullptr) {
        std::array<QImage, 2> drrImages{};
        if (m_apXrayView != nullptr) {
            drrImages[0] = m_apXrayView->renderedImage();
        }
        if (m_latXrayView != nullptr) {
            drrImages[1] = m_latXrayView->renderedImage();
        }
        m_sceneView->setDrrProjections(projections, projectionEnabled);
        m_sceneView->setDrrImages(std::move(drrImages));
        m_sceneView->refreshScene();
    }
}

void MprPlanVerificationWindow::refreshXrayInteractionOverlays()
{
    const std::string selectedId = selectedInstrumentId();
    const auto apPlacementConstraints = drrPlacementConstraintsForView(measurement::XrayPreset::AP);
    const auto latPlacementConstraints = drrPlacementConstraintsForView(measurement::XrayPreset::LAT);
    const bool latPlacementReady = latPlacementConstraints[0].has_value() && latPlacementConstraints[1].has_value();
    if (m_apXrayView != nullptr) {
        m_apXrayView->setPlan(&m_plan);
        m_apXrayView->setSelectedInstrumentId(selectedId);
        m_apXrayView->setPlacementActive(m_drrPlacementType.has_value() && !m_pendingDrrLines[0].has_value());
        m_apXrayView->setPendingLine(m_pendingDrrLines[0]);
        m_apXrayView->setPlacementConstraints(apPlacementConstraints);
        m_apXrayView->refreshOverlay();
    }
    if (m_latXrayView != nullptr) {
        m_latXrayView->setPlan(&m_plan);
        m_latXrayView->setSelectedInstrumentId(selectedId);
        m_latXrayView->setPlacementActive(
            m_drrPlacementType.has_value()
            && m_pendingDrrLines[0].has_value()
            && !m_pendingDrrLines[1].has_value()
            && latPlacementReady);
        m_latXrayView->setPendingLine(m_pendingDrrLines[1]);
        m_latXrayView->setPlacementConstraints(latPlacementConstraints);
        m_latXrayView->refreshOverlay();
    }
}

DrrUiSettings MprPlanVerificationWindow::drrSettingsFromControls(measurement::XrayPreset preset) const
{
    DrrUiSettings settings;
    const int index = xrayPresetIndex(preset);
    if (m_drrSid[index] == nullptr) {
        return settings;
    }

    settings.sidMm = m_drrSid[index]->value();
    settings.sodMm = std::min(m_drrSod[index]->value(), settings.sidMm - 1.0e-3);
    settings.detectorWidthMm = std::max(1.0, m_drrDetectorWidth[index]->value());
    settings.detectorHeightMm = std::max(1.0, m_drrDetectorHeight[index]->value());
    settings.pixelSpacingMm = m_drrPixelSpacing[index]->value();
    settings.rayStepMm = m_drrRayStep[index]->value();
    settings.windowCenter = m_drrWindowCenter[index]->value();
    settings.windowWidth = m_drrWindowWidth[index]->value();
    settings.gamma = m_drrGamma[index]->value();
    settings.huOffset = m_drrHuOffset[index]->value();
    settings.huScale = m_drrHuScale[index]->value();
    return settings;
}

void MprPlanVerificationWindow::setDrrPlacementMode(std::optional<measurement::InstrumentType> type)
{
    m_drrPlacementType = type;
    m_pendingDrrLines = {};
    if (m_drrPinButton != nullptr) {
        m_drrPinButton->setChecked(type.has_value() && *type == measurement::InstrumentType::GuidePin);
    }
    if (m_drrScrewButton != nullptr) {
        m_drrScrewButton->setChecked(type.has_value() && *type == measurement::InstrumentType::PedicleScrew);
    }
    statusBar()->showMessage(
        type.has_value()
            ? "DRR placement: draw one head-to-tail line in AP first, then refine it in constrained LAT."
            : "DRR placement cancelled.",
        5000);
    refreshXrayInteractionOverlays();
}

void MprPlanVerificationWindow::cancelDrrPlacement()
{
    setDrrPlacementMode(std::nullopt);
}

std::array<std::optional<DrrDetectorLine>, 2> MprPlanVerificationWindow::drrPlacementConstraintsForView(
    measurement::XrayPreset preset) const
{
    std::array<std::optional<DrrDetectorLine>, 2> constraints{};
    if (preset != measurement::XrayPreset::LAT || !m_drrPlacementType.has_value() || !m_pendingDrrLines[0].has_value()) {
        return constraints;
    }

    const DrrUiSettings apSettings = drrSettingsFromControls(measurement::XrayPreset::AP);
    const DrrUiSettings latSettings = drrSettingsFromControls(measurement::XrayPreset::LAT);
    measurement::ProjectionParams apProjection;
    measurement::ProjectionParams latProjection;
    measurement::DrrRenderSettings renderSettings;
    if (!buildDrrRenderRequest(&m_volume, measurement::XrayPreset::AP, apSettings, apProjection, renderSettings)
        || !buildDrrRenderRequest(&m_volume, measurement::XrayPreset::LAT, latSettings, latProjection, renderSettings)) {
        return constraints;
    }

    const auto headRay = detectorPixelToPatientRay(apProjection, m_pendingDrrLines[0]->head);
    const auto tailRay = detectorPixelToPatientRay(apProjection, m_pendingDrrLines[0]->tail);
    if (headRay.has_value()) {
        constraints[0] = projectPatientRayToDetectorConstraint(*headRay, latProjection);
    }
    if (tailRay.has_value()) {
        constraints[1] = projectPatientRayToDetectorConstraint(*tailRay, latProjection);
    }
    return constraints;
}

void MprPlanVerificationWindow::handleDrrLineCompleted(measurement::XrayPreset preset, DrrDetectorLine line)
{
    if (!m_drrPlacementType.has_value()) {
        return;
    }
    if (preset == measurement::XrayPreset::LAT && !m_pendingDrrLines[0].has_value()) {
        statusBar()->showMessage("DRR placement starts in AP. Draw AP first, then LAT will be constrained.", 5000);
        refreshXrayInteractionOverlays();
        return;
    }
    if (preset == measurement::XrayPreset::AP) {
        m_pendingDrrLines[0] = line;
        m_pendingDrrLines[1].reset();
        statusBar()->showMessage("AP DRR line fixed. Draw the LAT line on the magenta constrained rails.", 6000);
        refreshXrayInteractionOverlays();
        return;
    }

    const auto constraints = drrPlacementConstraintsForView(measurement::XrayPreset::LAT);
    if (!constraints[0].has_value() || !constraints[1].has_value()) {
        statusBar()->showMessage("LAT constraint is not available for this AP line. Cancel and redraw AP.", 7000);
        refreshXrayInteractionOverlays();
        return;
    }

    line.head = closestDetectorPointOnSegment(line.head, *constraints[0]);
    line.tail = closestDetectorPointOnSegment(line.tail, *constraints[1]);
    m_pendingDrrLines[1] = line;
    tryCreateInstrumentFromDrrLines();
    refreshXrayInteractionOverlays();
}

void MprPlanVerificationWindow::tryCreateInstrumentFromDrrLines()
{
    if (!m_drrPlacementType.has_value() || !m_pendingDrrLines[0].has_value() || !m_pendingDrrLines[1].has_value()) {
        return;
    }

    const DrrUiSettings apSettings = drrSettingsFromControls(measurement::XrayPreset::AP);
    const DrrUiSettings latSettings = drrSettingsFromControls(measurement::XrayPreset::LAT);
    measurement::ProjectionParams apProjection;
    measurement::ProjectionParams latProjection;
    measurement::DrrRenderSettings renderSettings;
    if (!buildDrrRenderRequest(&m_volume, measurement::XrayPreset::AP, apSettings, apProjection, renderSettings)
        || !buildDrrRenderRequest(&m_volume, measurement::XrayPreset::LAT, latSettings, latProjection, renderSettings)) {
        statusBar()->showMessage("DRR placement failed: projection geometry is not ready.", 6000);
        return;
    }

    const auto apHeadRay = detectorPixelToPatientRay(apProjection, m_pendingDrrLines[0]->head);
    const auto latHeadRay = detectorPixelToPatientRay(latProjection, m_pendingDrrLines[1]->head);
    const auto apTailRay = detectorPixelToPatientRay(apProjection, m_pendingDrrLines[0]->tail);
    const auto latTailRay = detectorPixelToPatientRay(latProjection, m_pendingDrrLines[1]->tail);
    if (!apHeadRay.has_value() || !latHeadRay.has_value() || !apTailRay.has_value() || !latTailRay.has_value()) {
        statusBar()->showMessage("DRR placement failed: one of the drawn lines is outside the detector.", 6000);
        return;
    }

    const auto head = closestPointBetweenRays(*apHeadRay, *latHeadRay);
    const auto tail = closestPointBetweenRays(*apTailRay, *latTailRay);
    constexpr double kMaxRayPairDistanceMm = 25.0;
    if (!head.has_value() || !tail.has_value()
        || head->distanceMm > kMaxRayPairDistanceMm
        || tail->distanceMm > kMaxRayPairDistanceMm) {
        m_pendingDrrLines[1].reset();
        statusBar()->showMessage("DRR AP/LAT lines do not meet in 3D. Redraw the mismatched line.", 7000);
        return;
    }

    const measurement::Vec3d direction = measurement::normalize(tail->pointPatientMm - head->pointPatientMm);
    const double lengthMm = measurement::length(tail->pointPatientMm - head->pointPatientMm);
    if (!isFiniteVec(direction) || lengthMm <= 1.0) {
        statusBar()->showMessage("DRR placement failed: reconstructed instrument is too short.", 6000);
        return;
    }

    const measurement::InstrumentType type = *m_drrPlacementType;
    const std::string id = (type == measurement::InstrumentType::GuidePin ? "pin-" : "screw-")
        + std::to_string(m_nextInstrumentIndex++);
    const double diameterMm = type == measurement::InstrumentType::GuidePin ? 2.0 : 6.5;
    const auto result = type == measurement::InstrumentType::GuidePin
        ? m_planController->createGuidePin(id, head->pointPatientMm, direction, lengthMm, diameterMm, id)
        : m_planController->createPedicleScrew(id, head->pointPatientMm, direction, lengthMm, diameterMm, id);
    if (!result.ok()) {
        statusBar()->showMessage(QString::fromStdString(result.error().message), 6000);
        return;
    }

    m_pendingDrrLines = {};
    m_drrPlacementType.reset();
    if (m_drrPinButton != nullptr) {
        m_drrPinButton->setChecked(false);
    }
    if (m_drrScrewButton != nullptr) {
        m_drrScrewButton->setChecked(false);
    }
    m_placementController->setSelectedInstrumentId(id);
    refreshInstrumentList();
    selectInstrumentById(id);
    (void)jumpToInstrumentPlanningPose(id);
    syncSpinBoxesFromSelectedInstrument();
    statusBar()->showMessage("DRR instrument created from the unified AP/LAT plan.", 5000);
    refreshAll(true);
}

void MprPlanVerificationWindow::handleDrrInstrumentSelected(std::string id)
{
    selectInstrumentById(id);
    const bool jumped = jumpToInstrumentPlanningPose(id);
    if (!jumped) {
        const std::array<IMprSliceView*, 3> views{m_axialView, m_sagittalView, m_coronalView};
        for (size_t index = 0; index < views.size(); ++index) {
            views[index]->setSelectedInstrumentId(id);
            views[index]->refreshImage();
        }
    }
    refreshXrayInteractionOverlays();
    refreshPlanScene();
    refreshStatus();
}

void MprPlanVerificationWindow::handleDrrInstrumentDragged(
    measurement::XrayPreset preset,
    std::string id,
    DrrInteractionTarget target,
    DrrDetectorPoint detectorPoint)
{
    measurement::Instrument* instrument = m_plan.findInstrument(id);
    if (instrument == nullptr) {
        return;
    }
    if (instrument->locked) {
        statusBar()->showMessage("Locked instruments cannot be edited from DRR.", 4000);
        return;
    }

    const DrrUiSettings settings = drrSettingsFromControls(preset);
    measurement::ProjectionParams projection;
    measurement::DrrRenderSettings renderSettings;
    if (!buildDrrRenderRequest(&m_volume, preset, settings, projection, renderSettings)) {
        return;
    }
    const auto ray = detectorPixelToPatientRay(projection, detectorPoint);
    if (!ray.has_value()) {
        return;
    }

    measurement::InstrumentPatch patch;
    patch.entryPointPatientMm = instrument->entryPointPatientMm;
    patch.directionPatientUnit = instrument->directionPatientUnit;
    patch.lengthMm = instrument->lengthMm;
    patch.diameterMm = instrument->diameterMm;
    patch.visible = instrument->visible;
    patch.locked = instrument->locked;
    patch.label = instrument->label;

    if (target == DrrInteractionTarget::Head) {
        const measurement::Vec3d detectorNormal = measurement::normalize(
            measurement::cross(projection.detectorUPatientUnit, projection.detectorVPatientUnit));
        const auto movedHead = rayPlaneIntersection(*ray, instrument->entryPointPatientMm, detectorNormal);
        if (!movedHead.has_value()) {
            return;
        }
        patch.entryPointPatientMm = *movedHead;
    } else if (target == DrrInteractionTarget::Tail) {
        std::optional<measurement::Vec3d> movedTail = raySphereIntersectionNearDirection(
            *ray,
            instrument->entryPointPatientMm,
            instrument->lengthMm,
            instrument->directionPatientUnit);
        if (!movedTail.has_value()) {
            movedTail = closestPointOnRay(*ray, measurement::endpointPatientMm(*instrument));
        }
        const measurement::Vec3d newDirection = measurement::normalize(*movedTail - instrument->entryPointPatientMm);
        if (!isFiniteVec(newDirection) || measurement::length(newDirection) <= 1.0e-6) {
            return;
        }
        patch.directionPatientUnit = newDirection;
    } else {
        return;
    }

    const auto result = m_planController->updateInstrument(id, patch);
    if (!result.ok()) {
        statusBar()->showMessage(QString::fromStdString(result.error().message), 6000);
        return;
    }

    selectInstrumentById(id);
    syncSpinBoxesFromSelectedInstrument();
    const std::array<IMprSliceView*, 3> views{m_axialView, m_sagittalView, m_coronalView};
    for (size_t index = 0; index < views.size(); ++index) {
        IMprSliceView* view = views[index];
        view->setVolume(&m_volume);
        view->setLinkedPlaneFrames(&m_planeFrames);
        view->setState(&m_viewStates[index]);
        view->setPlan(&m_plan);
        view->setSelectedInstrumentId(id);
        view->refreshImage();
    }
    refreshPlanScene();
    refreshXrayInteractionOverlays();
    refreshStatus();
}

}  // namespace measurement_app
