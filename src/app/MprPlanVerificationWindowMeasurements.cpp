#include "MprPlanVerificationWindow.h"

#include "MprSliceWidget.h"

#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QStatusBar>

#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace measurement_app {
namespace {

[[nodiscard]] bool isFiniteVec(measurement::Vec3d value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

[[nodiscard]] measurement::MeasurementViewType measurementViewTypeForPlane(measurement::MprPlane plane)
{
    switch (plane) {
    case measurement::MprPlane::Axial:
        return measurement::MeasurementViewType::Axial;
    case measurement::MprPlane::Sagittal:
        return measurement::MeasurementViewType::Sagittal;
    case measurement::MprPlane::Coronal:
        return measurement::MeasurementViewType::Coronal;
    }
    return measurement::MeasurementViewType::Oblique;
}

[[nodiscard]] std::optional<measurement::MprPlane> planeForMeasurementViewType(measurement::MeasurementViewType viewType)
{
    switch (viewType) {
    case measurement::MeasurementViewType::Axial:
        return measurement::MprPlane::Axial;
    case measurement::MeasurementViewType::Sagittal:
        return measurement::MprPlane::Sagittal;
    case measurement::MeasurementViewType::Coronal:
        return measurement::MprPlane::Coronal;
    case measurement::MeasurementViewType::Oblique:
        return std::nullopt;
    }
    return std::nullopt;
}

[[nodiscard]] const char* measurementTypeName(measurement::MeasurementType type)
{
    return type == measurement::MeasurementType::Angle ? "角度" : "距离";
}

}  // namespace

void MprPlanVerificationWindow::refreshMeasurementOverlays()
{
    const auto pendingPlane = m_pendingMeasurementPlane;
    const std::vector<measurement::Vec3d>& pendingPoints = m_measurementStateMachine.pendingPoints();
    const auto updateView = [&](MprSliceWidget* view) {
        if (view == nullptr) {
            return;
        }

        std::vector<measurement::Vec3d> viewPendingPoints;
        std::optional<measurement::Vec3d> viewHoverPoint;
        if (pendingPlane.has_value() && view->plane() == *pendingPlane) {
            viewPendingPoints = pendingPoints;
            viewHoverPoint = m_measurementHoverPatientMm;
        }
        view->setMeasurementInteractionState(m_measurementMode, std::move(viewPendingPoints), viewHoverPoint, m_selectedMeasurementId);
    };

    updateView(m_axialView);
    updateView(m_sagittalView);
    updateView(m_coronalView);
}

void MprPlanVerificationWindow::refreshMeasurementList()
{
    if (m_measurementList == nullptr || m_measurementLabel == nullptr) {
        return;
    }

    if (m_selectedMeasurementId.isValid() && !m_measurementStore.find(m_selectedMeasurementId).has_value()) {
        m_selectedMeasurementId = measurement::MeasurementId();
    }

    m_measurementList->blockSignals(true);
    m_measurementList->clear();
    int selectedRow = -1;
    int row = 0;
    for (const measurement::MeasurementAnnotation& annotation : m_measurementStore.all()) {
        const QString typeText = QString::fromUtf8(measurementTypeName(annotation.type));
        const QString value = QString::fromStdString(annotation.measurementText());
        const QString primary = annotation.label.empty()
            ? value
            : QString::fromStdString(annotation.label);
        const QString secondary = annotation.label.empty()
            ? typeText
            : QString("%1  %2").arg(typeText, value);
        const QString itemText = QString("%1\n%2").arg(primary, secondary);
        auto* item = new QListWidgetItem(itemText, m_measurementList);
        item->setData(Qt::UserRole, static_cast<qlonglong>(annotation.id.value()));
        item->setToolTip(QString("#%1  %2").arg(annotation.id.value()).arg(typeText));
        item->setSizeHint(QSize(0, 54));
        if (annotation.id == m_selectedMeasurementId) {
            selectedRow = row;
        }
        ++row;
    }
    if (selectedRow >= 0) {
        m_measurementList->setCurrentRow(selectedRow);
    }
    m_measurementList->blockSignals(false);

    const auto selected = m_measurementStore.find(m_selectedMeasurementId);
    m_measurementLabel->blockSignals(true);
    m_measurementLabel->setEnabled(selected.has_value());
    m_measurementLabel->setText(selected.has_value() ? QString::fromStdString(selected->label) : QString());
    m_measurementLabel->blockSignals(false);
}

void MprPlanVerificationWindow::setMeasurementMode(measurement::MeasurementMode mode)
{
    m_measurementMode = mode;
    m_measurementStateMachine.setMode(mode);
    m_pendingMeasurementPlane.reset();
    m_measurementHoverPatientMm.reset();

    if (m_measureNavigateButton != nullptr) {
        m_measureNavigateButton->setChecked(mode == measurement::MeasurementMode::Navigate);
    }
    if (m_measureDistanceButton != nullptr) {
        m_measureDistanceButton->setChecked(mode == measurement::MeasurementMode::Distance);
    }
    if (m_measureAngleButton != nullptr) {
        m_measureAngleButton->setChecked(mode == measurement::MeasurementMode::Angle);
    }

    refreshMeasurementOverlays();
    refreshStatus();
}

void MprPlanVerificationWindow::handleMeasurementPointAdded(
    measurement::MprPlane plane,
    measurement::Vec3d patientPoint,
    measurement::MeasurementPlane slicePlane)
{
    if (m_measurementMode == measurement::MeasurementMode::Navigate) {
        return;
    }

    activateMprPlane(plane);
    if (!m_pendingMeasurementPlane.has_value() || *m_pendingMeasurementPlane != plane) {
        m_measurementStateMachine.reset();
        m_pendingMeasurementPlane = plane;
    }

    const size_t previousPointCount = m_measurementStateMachine.pendingPoints().size();
    measurement::MeasurementAnnotation completed;
    if (m_measurementStateMachine.addPoint(patientPoint, completed)) {
        completed.createdPlane = slicePlane;
        completed.createdViewType = measurementViewTypeForPlane(plane);
        const measurement::MeasurementId id = m_measurementStore.add(std::move(completed));
        m_pendingMeasurementPlane.reset();
        m_measurementHoverPatientMm.reset();
        selectMeasurementById(id);
        statusBar()->showMessage("已添加测量。", 3000);
    } else {
        m_measurementHoverPatientMm = patientPoint;
        const size_t requiredPointCount = m_measurementMode == measurement::MeasurementMode::Angle ? 4U : 2U;
        if (previousPointCount + 1U >= requiredPointCount && m_measurementStateMachine.pendingPoints().empty()) {
            statusBar()->showMessage("选取点退化，测量已被拒绝。", 5000);
        }
    }

    refreshMeasurementOverlays();
    refreshMeasurementList();
    refreshStatus();
}

void MprPlanVerificationWindow::handleMeasurementHoverChanged(
    measurement::MprPlane plane,
    std::optional<measurement::Vec3d> patientPoint)
{
    if (!m_pendingMeasurementPlane.has_value() || *m_pendingMeasurementPlane != plane) {
        return;
    }

    m_measurementHoverPatientMm = patientPoint;
    refreshMeasurementOverlays();
}

void MprPlanVerificationWindow::cancelPendingMeasurement()
{
    m_measurementStateMachine.reset();
    m_pendingMeasurementPlane.reset();
    m_measurementHoverPatientMm.reset();
    refreshMeasurementOverlays();
    refreshStatus();
}

void MprPlanVerificationWindow::selectMeasurementById(measurement::MeasurementId id)
{
    if (!id.isValid()) {
        return;
    }

    m_selectedMeasurementId = id;
    for (const measurement::MeasurementAnnotation& annotation : m_measurementStore.all()) {
        measurement::MeasurementAnnotation updated = annotation;
        const bool shouldSelect = annotation.id == id;
        if (updated.selected != shouldSelect) {
            updated.selected = shouldSelect;
            (void)m_measurementStore.update(updated);
        }
    }
    refreshMeasurementOverlays();
    refreshMeasurementList();
}

void MprPlanVerificationWindow::jumpToMeasurement(measurement::MeasurementId id)
{
    if (!id.isValid() || !m_volume.image) {
        return;
    }

    const auto annotation = m_measurementStore.find(id);
    const auto anchor = m_measurementStore.anchorWorldPoint(id);
    if (!annotation.has_value() || !anchor.has_value() || !isFiniteVec(*anchor)) {
        statusBar()->showMessage("无法定位选中的测量。", 4000);
        return;
    }

    if (const auto plane = planeForMeasurementViewType(annotation->createdViewType)) {
        activateMprPlane(*plane);
    }
    setCrosshairPatient(*anchor);
    statusBar()->showMessage(QString("已定位测量 #%1。").arg(id.value()), 3000);
}

void MprPlanVerificationWindow::deleteSelectedMeasurement()
{
    const auto id = selectedMeasurementId();
    if (!id.has_value()) {
        return;
    }

    (void)m_measurementStore.remove(*id);
    m_selectedMeasurementId = measurement::MeasurementId();
    cancelPendingMeasurement();
    refreshMeasurementList();
    refreshStatus();
}

void MprPlanVerificationWindow::clearMeasurements()
{
    m_measurementStore.clear();
    m_selectedMeasurementId = measurement::MeasurementId();
    cancelPendingMeasurement();
    refreshMeasurementList();
    refreshStatus();
}

void MprPlanVerificationWindow::renameSelectedMeasurement()
{
    if (m_measurementLabel == nullptr) {
        return;
    }

    const auto id = selectedMeasurementId();
    if (!id.has_value()) {
        return;
    }

    (void)m_measurementStore.rename(*id, m_measurementLabel->text().trimmed().toStdString());
    refreshMeasurementOverlays();
    refreshMeasurementList();
}

std::optional<measurement::MeasurementId> MprPlanVerificationWindow::selectedMeasurementId() const
{
    if (m_measurementList == nullptr || m_measurementList->currentItem() == nullptr) {
        return std::nullopt;
    }

    const qlonglong id = m_measurementList->currentItem()->data(Qt::UserRole).toLongLong();
    if (id < 0) {
        return std::nullopt;
    }
    return measurement::MeasurementId(static_cast<std::int64_t>(id));
}

}  // namespace measurement_app
