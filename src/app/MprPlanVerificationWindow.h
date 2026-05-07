#pragma once

#include "DrrInteractionGeometry.h"
#include "DrrRenderRequest.h"
#include "InstrumentRenderModel.h"
#include "measurement/core/Instrument.h"
#include "measurement/core/MeasurementAnnotation.h"
#include "measurement/core/MeasurementStateMachine.h"
#include "measurement/core/MeasurementStore.h"
#include "measurement/core/Volume.h"
#include "measurement/core/Xray.h"
#include "measurement/mpr/MprResliceEngine.h"
#include "measurement/persistence/ProjectManifest.h"
#include "measurement/planning/InstrumentGeometry.h"
#include "measurement/vtk/VtkMprResliceAdapter.h"

#include <QImage>
#include <QMainWindow>
#include <QPointF>
#include <QPointer>
#include <QSize>
#include <QString>
#include <QWidget>

#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QSlider;
class QWheelEvent;

namespace measurement_app {

class IMprSliceView {
public:
    virtual ~IMprSliceView() = default;

    [[nodiscard]] virtual measurement::MprPlane plane() const = 0;
    virtual void setVolume(const measurement::VolumeData* volume) = 0;
    virtual void setState(const measurement::MprViewState* state) = 0;
    virtual void setLinkedPlaneFrames(const std::array<measurement::MprSliceFrame, 3>* frames) = 0;
    virtual void setPlan(const measurement::SurgicalPlan* plan) = 0;
    virtual void setSelectedInstrumentId(std::string id) = 0;
    virtual void refreshImage() = 0;
};

class IPlanSceneView {
public:
    virtual ~IPlanSceneView() = default;

    virtual void setVolume(const measurement::VolumeData* volume) = 0;
    virtual void setPlan(const measurement::SurgicalPlan* plan) = 0;
    virtual void setSelectedInstrumentId(std::string id) = 0;
    virtual void refreshScene() = 0;
};

class MprSliceWidget final : public QWidget, public IMprSliceView {
public:
    explicit MprSliceWidget(measurement::MprPlane plane, QWidget* parent = nullptr);

    [[nodiscard]] measurement::MprPlane plane() const override;
    void setVolume(const measurement::VolumeData* volume) override;
    void setState(const measurement::MprViewState* state) override;
    void setLinkedPlaneFrames(const std::array<measurement::MprSliceFrame, 3>* frames) override;
    void setPlan(const measurement::SurgicalPlan* plan) override;
    void setRequest(measurement::MprSliceRequest request);
    void setSelectedInstrumentId(std::string id) override;
    void refreshImage() override;
    void setCrosshairChangedCallback(std::function<void(measurement::Vec3d)> callback);
    void setWindowLevelChangedCallback(std::function<void(double, double)> callback);
    void setPlaneRotationCallback(std::function<void(measurement::MprPlane, measurement::MprPlane, double)> callback);
    void setActivatedCallback(std::function<void(measurement::MprPlane)> callback);
    void setMeasurements(const std::vector<measurement::MeasurementAnnotation>* measurements);
    void setMeasurementInteractionState(
        measurement::MeasurementMode mode,
        std::vector<measurement::Vec3d> pendingPoints,
        std::optional<measurement::Vec3d> hoverPoint,
        measurement::MeasurementId selectedId);
    void setMeasurementPointAddedCallback(
        std::function<void(measurement::MprPlane, measurement::Vec3d, measurement::MeasurementPlane)> callback);
    void setMeasurementHoverChangedCallback(
        std::function<void(measurement::MprPlane, std::optional<measurement::Vec3d>)> callback);
    void setMeasurementCancelCallback(std::function<void()> callback);
    void resetViewPresentation();

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    QSize minimumSizeHint() const override;

private:
    enum class InteractionTarget {
        None,
        Center,
        FirstHandle,
        SecondHandle,
    };

    enum class DragMode {
        None,
        CrosshairCenter,
        RotateFirstLine,
        RotateSecondLine,
        Pan,
        Zoom,
        WindowLevel,
    };

    struct CrosslineInfo {
        measurement::MprPlane sourcePlane = measurement::MprPlane::Axial;
        QPointF centerImage;
        QPointF directionImage;
        QPointF handleImage;
        measurement::Vec3d directionPatientUnit{};
    };

    [[nodiscard]] measurement::Result<measurement::MprResliceParameters> parameters() const;
    [[nodiscard]] measurement::MprViewState stateForPlane() const;
    [[nodiscard]] QPointF patientToImagePoint(measurement::Vec3d patient) const;
    [[nodiscard]] measurement::Vec3d imagePointToPatient(QPointF imagePoint) const;
    [[nodiscard]] QRect imageRect() const;
    [[nodiscard]] QPointF widgetPointToImagePoint(const QPoint& position) const;
    [[nodiscard]] InteractionTarget hitTestCrosshair(const QPoint& position) const;
    [[nodiscard]] QPointF clampImagePoint(QPointF imagePoint) const;
    [[nodiscard]] std::array<CrosslineInfo, 2> crosslines() const;
    [[nodiscard]] QPointF imageDirectionForPlane(const measurement::MprSliceFrame& currentFrame, measurement::Vec3d otherPlaneNormal) const;
    [[nodiscard]] QPointF visibleHandlePoint(QPointF centerImage, QPointF directionImage) const;
    [[nodiscard]] std::pair<QPointF, QPointF> visibleCrosslineEndpoints(const CrosslineInfo& line) const;
    [[nodiscard]] std::pair<QPointF, QPointF> crosslineHandleCenters(const CrosslineInfo& line) const;
    [[nodiscard]] std::optional<measurement::MeasurementPlane> currentMeasurementPlane() const;
    [[nodiscard]] std::optional<measurement::Vec3d> patientPointFromWidgetPosition(const QPoint& position) const;
    void drawCrossline(QPainter& painter, const CrosslineInfo& line, QColor color, bool drawHandle, bool drawDirectionCue = false);
    void drawCrosslineOrientationLabels(QPainter& painter, const CrosslineInfo& line, QColor color);
    void drawRotationHandle(QPainter& painter, QPointF centerImage, QPointF directionImage, QColor color);
    void drawInstrumentDirectionCue(QPainter& painter, QPointF handleCenterImage, QPointF directionImage, QColor color);
    void drawMeasurementOverlays(QPainter& painter);
    void drawMeasurementAnnotation(QPainter& painter, const measurement::MeasurementVisibilityResult& result);
    void drawMeasurementPreview(QPainter& painter);
    void anchorCrosshairAtImagePoint(QPointF imagePoint);
    void updateCursorForHover(const QPoint& position);
    void beginCrosshairDrag(InteractionTarget target, const QPoint& position);
    void updateCrosshairDrag(const QPoint& position);
    void beginPanDrag(const QPoint& position);
    void updatePanDrag(const QPoint& position);
    void beginZoomDrag(const QPoint& position);
    void updateZoomDrag(const QPoint& position);
    void beginWindowLevelDrag(const QPoint& position);
    void updateWindowLevelDrag(const QPoint& position);
    void beginRotationDrag(InteractionTarget target, const QPoint& position);
    void updateRotationDrag(const QPoint& position);
    void stepSlice(int steps);
    void drawInstrumentOverlays(QPainter& painter);
    [[nodiscard]] QImage imageFromVtkReslice(vtkImageData& image) const;

    measurement::MprPlane m_plane = measurement::MprPlane::Axial;
    const measurement::VolumeData* m_volume = nullptr;
    std::string m_volumeSignature;
    const measurement::MprViewState* m_state = nullptr;
    const std::array<measurement::MprSliceFrame, 3>* m_linkedPlaneFrames = nullptr;
    const measurement::SurgicalPlan* m_plan = nullptr;
    measurement::MprSliceRequest m_request;
    measurement::VtkMprResliceAdapter m_resliceAdapter;
    std::string m_selectedInstrumentId;
    std::vector<InstrumentRenderSection> m_instrumentSections;
    const std::vector<measurement::MeasurementAnnotation>* m_measurements = nullptr;
    measurement::MeasurementMode m_measurementMode = measurement::MeasurementMode::Navigate;
    std::vector<measurement::Vec3d> m_pendingMeasurementPoints;
    std::optional<measurement::Vec3d> m_measurementHoverPatientMm;
    measurement::MeasurementId m_selectedMeasurementId;
    QImage m_image;
    QString m_renderStatus;
    std::function<void(measurement::Vec3d)> m_crosshairChanged;
    std::function<void(double, double)> m_windowLevelChanged;
    std::function<void(measurement::MprPlane, measurement::MprPlane, double)> m_planeRotationChanged;
    std::function<void(measurement::MprPlane)> m_activated;
    std::function<void(measurement::MprPlane, measurement::Vec3d, measurement::MeasurementPlane)> m_measurementPointAdded;
    std::function<void(measurement::MprPlane, std::optional<measurement::Vec3d>)> m_measurementHoverChanged;
    std::function<void()> m_measurementCancelRequested;
    double m_zoom = 1.0;
    measurement::Vec3d m_pan{};
    double m_windowCenterHu = 400.0;
    double m_windowWidthHu = 2000.0;
    DragMode m_dragMode = DragMode::None;
    QPoint m_lastMousePosition;
    QPoint m_dragStartPosition;
    measurement::Vec3d m_dragStartCrosshairPatientMm{};
    measurement::Vec3d m_dragStartPan{};
    double m_dragStartZoom = 1.0;
    double m_dragStartWindowCenterHu = 400.0;
    double m_dragStartWindowWidthHu = 2000.0;
    double m_lastRotationAngleRad = 0.0;
};

class PlanSceneWidget final : public QWidget, public IPlanSceneView {
public:
    explicit PlanSceneWidget(QWidget* parent = nullptr);
    ~PlanSceneWidget() override;

    void setVolume(const measurement::VolumeData* volume) override;
    void setPlan(const measurement::SurgicalPlan* plan) override;
    void setSelectedInstrumentId(std::string id) override;
    void setDrrProjections(std::array<measurement::ProjectionParams, 2> projections, std::array<bool, 2> enabled);
    void setDrrImages(std::array<QImage, 2> images);
    void refreshScene() override;
    void resetCamera();

protected:
    QSize minimumSizeHint() const override;

private:
    struct Impl;

    [[nodiscard]] std::string volumeSignature() const;
    void rebuildScene();

    std::unique_ptr<Impl> m_impl;
    const measurement::VolumeData* m_volume = nullptr;
    const measurement::SurgicalPlan* m_plan = nullptr;
    std::string m_selectedInstrumentId;
    std::array<measurement::ProjectionParams, 2> m_drrProjections{};
    std::array<bool, 2> m_drrProjectionEnabled{false, false};
    std::array<QImage, 2> m_drrImages{};
};

enum class DrrInteractionTarget {
    None,
    Head,
    Tail,
    Body
};

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
