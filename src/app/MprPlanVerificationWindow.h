#pragma once

#include "DrrInteractionGeometry.h"
#include "DrrRenderRequest.h"
#include "measurement/core/Instrument.h"
#include "measurement/core/MeasurementAnnotation.h"
#include "measurement/core/MeasurementStateMachine.h"
#include "measurement/core/MeasurementStore.h"
#include "measurement/core/Volume.h"
#include "measurement/core/Xray.h"
#include "measurement/mpr/MprResliceEngine.h"
#include "measurement/persistence/ProjectManifest.h"
#include "measurement/planning/InstrumentGeometry.h"

#include <QMainWindow>
#include <QPointer>
#include <QString>

#include <array>
#include <memory>
#include <optional>
#include <string>

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QSlider;
class QWheelEvent;

namespace measurement_app {

class IMprSliceView;
class MprSliceWidget;
class PlanSceneWidget;
class XrayDisplayWidget;

class MprPlanVerificationWindow final : public QMainWindow {
public:
    explicit MprPlanVerificationWindow(QWidget* parent = nullptr);

private:
    void buildUi();
    void loadStartupVolume();
    void loadSyntheticVolume();
    void loadDicomFolder();
    [[nodiscard]] bool tryLoadDicomFolder(const QString& folder, QString* failureMessage = nullptr);
    void activateLoadedVolumeData();
    void saveProject();
    void resetCrosshairToVolumeCenter(bool refreshViews = true);
    void setCrosshairPatient(measurement::Vec3d patient);
    void setCrosshairVoxel(measurement::Vec3d voxel, bool refreshViews = true);
    void setWindowLevel(double centerHu, double widthHu);
    void resetAllViews();
    void resetPatientPositionControls();
    void applyPatientPosition(bool prone, bool feetFirst);
    void initializePlaneFrames();
    void syncPlaneFrameOrigins();
    void syncPerViewStates();
    void rotateCrosshairPlane(measurement::MprPlane aroundPlane, measurement::MprPlane linePlane, double deltaAngleRad);
    void setFreeObliqueMode(bool enabled);
    void orthogonalizePlaneFrames(measurement::MprPlane anchorPlane, measurement::MprPlane linePlane);
    void syncSlidersFromCrosshair();
    void syncSpinBoxesFromSelectedInstrument();
    void refreshAll(bool refreshScene = false);
    void refreshPlanScene();
    void refreshXrayViews();
    void refreshXrayInteractionOverlays();
    void refreshMeasurementOverlays();
    void refreshMeasurementList();
    void setMeasurementMode(measurement::MeasurementMode mode);
    void handleMeasurementPointAdded(
        measurement::MprPlane plane,
        measurement::Vec3d patientPoint,
        measurement::MeasurementPlane slicePlane);
    void handleMeasurementHoverChanged(measurement::MprPlane plane, std::optional<measurement::Vec3d> patientPoint);
    void cancelPendingMeasurement();
    void selectMeasurementById(measurement::MeasurementId id);
    void jumpToMeasurement(measurement::MeasurementId id);
    void deleteSelectedMeasurement();
    void clearMeasurements();
    void renameSelectedMeasurement();
    [[nodiscard]] std::optional<measurement::MeasurementId> selectedMeasurementId() const;
    [[nodiscard]] DrrUiSettings drrSettingsFromControls(measurement::XrayPreset preset) const;
    void setDrrPlacementMode(std::optional<measurement::InstrumentType> type);
    void cancelDrrPlacement();
    [[nodiscard]] std::array<std::optional<DrrDetectorLine>, 2> drrPlacementConstraintsForView(
        measurement::XrayPreset preset) const;
    void handleDrrLineCompleted(measurement::XrayPreset preset, DrrDetectorLine line);
    void tryCreateInstrumentFromDrrLines();
    void handleDrrInstrumentSelected(std::string id);
    void handleDrrInstrumentDragged(
        measurement::XrayPreset preset,
        std::string id,
        DrrInteractionTarget target,
        DrrDetectorPoint detectorPoint);
    void selectInstrumentById(const std::string& id);
    void refreshStatus();
    void refreshInstrumentList();
    void addInstrument(measurement::InstrumentType type);
    void applyInstrumentPropertyEdits();
    void removeSelectedInstrument();
    [[nodiscard]] bool jumpToInstrumentPlanningPose(const std::string& id);
    void toggleInstrumentEdit();
    void beginInstrumentEdit(const std::string& id);
    [[nodiscard]] bool requestFinishInstrumentEdit();
    void finishInstrumentEdit(bool saveChanges);
    void updateInstrumentEditButton();
    void activateMprPlane(measurement::MprPlane plane);
    void syncPlacementSelectionFromUi();
    void alignEditingInstrumentToCrosshairLine(measurement::MprPlane viewPlane, measurement::MprPlane linePlane);
    [[nodiscard]] measurement::Vec3d crosshairLineDirectionPatient(measurement::MprPlane viewPlane, measurement::MprPlane linePlane) const;
    [[nodiscard]] measurement::Vec3d activeCrosshairLineDirectionPatient() const;
    [[nodiscard]] std::string selectedInstrumentId() const;
    [[nodiscard]] measurement::InstrumentPatch patchFromControls() const;
    [[nodiscard]] measurement::ProjectManifest makeManifest() const;

    measurement::VolumeData m_volume;
    measurement::SurgicalPlan m_plan;
    measurement::MeasurementStore m_measurementStore;
    measurement::MeasurementStateMachine m_measurementStateMachine;
    std::unique_ptr<measurement::InstrumentPlanController> m_planController;
    std::unique_ptr<measurement::InstrumentPlacementController> m_placementController;
    measurement::MprViewState m_mprState;
    std::array<measurement::MprSliceFrame, 3> m_planeFrames{};
    std::array<measurement::MprViewState, 3> m_viewStates{};
    measurement::MprPlane m_activeMprPlane = measurement::MprPlane::Axial;
    measurement::MprPlane m_activeCrosshairLinePlane = measurement::MprPlane::Sagittal;
    std::optional<measurement::MprPlane> m_pendingMeasurementPlane;
    std::optional<measurement::Vec3d> m_measurementHoverPatientMm;
    measurement::MeasurementId m_selectedMeasurementId;
    measurement::MeasurementMode m_measurementMode = measurement::MeasurementMode::Navigate;
    int m_nextInstrumentIndex = 1;
    bool m_syncingControls = false;
    bool m_freeObliqueMode = true;
    bool m_instrumentEditActive = false;
    std::string m_editingInstrumentId;
    measurement::InstrumentPatch m_editOriginalPatch;

    MprSliceWidget* m_axialView = nullptr;
    MprSliceWidget* m_sagittalView = nullptr;
    MprSliceWidget* m_coronalView = nullptr;
    PlanSceneWidget* m_sceneView = nullptr;
    XrayDisplayWidget* m_apXrayView = nullptr;
    XrayDisplayWidget* m_latXrayView = nullptr;
    std::array<QDoubleSpinBox*, 2> m_drrSid{};
    std::array<QDoubleSpinBox*, 2> m_drrSod{};
    std::array<QDoubleSpinBox*, 2> m_drrDetectorWidth{};
    std::array<QDoubleSpinBox*, 2> m_drrDetectorHeight{};
    std::array<QDoubleSpinBox*, 2> m_drrPixelSpacing{};
    std::array<QDoubleSpinBox*, 2> m_drrRayStep{};
    std::array<QDoubleSpinBox*, 2> m_drrWindowCenter{};
    std::array<QDoubleSpinBox*, 2> m_drrWindowWidth{};
    std::array<QDoubleSpinBox*, 2> m_drrGamma{};
    std::array<QDoubleSpinBox*, 2> m_drrHuOffset{};
    std::array<QDoubleSpinBox*, 2> m_drrHuScale{};
    QPushButton* m_loadDicomButton = nullptr;
    QPushButton* m_drrPinButton = nullptr;
    QPushButton* m_drrScrewButton = nullptr;
    QPushButton* m_drrCancelButton = nullptr;
    std::optional<measurement::InstrumentType> m_drrPlacementType;
    std::array<std::optional<DrrDetectorLine>, 2> m_pendingDrrLines{};
    QLabel* m_statusLabel = nullptr;
    QLabel* m_volumeLabel = nullptr;
    QListWidget* m_measurementList = nullptr;
    QLineEdit* m_measurementLabel = nullptr;
    QPushButton* m_measureNavigateButton = nullptr;
    QPushButton* m_measureDistanceButton = nullptr;
    QPushButton* m_measureAngleButton = nullptr;
    QComboBox* m_patientPostureCombo = nullptr;
    QComboBox* m_headFeetDirectionCombo = nullptr;
    QListWidget* m_instrumentList = nullptr;
    QSlider* m_xSlider = nullptr;
    QSlider* m_ySlider = nullptr;
    QSlider* m_zSlider = nullptr;
    QLabel* m_xValueLabel = nullptr;
    QLabel* m_yValueLabel = nullptr;
    QLabel* m_zValueLabel = nullptr;
    QLineEdit* m_label = nullptr;
    QDoubleSpinBox* m_length = nullptr;
    QDoubleSpinBox* m_diameter = nullptr;
    QPushButton* m_freeObliqueButton = nullptr;
    QPushButton* m_editInstrumentButton = nullptr;
    bool m_appliedPatientProne = false;
    bool m_appliedFeetFirst = false;
};

}  // namespace measurement_app
