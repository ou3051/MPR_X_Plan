#pragma once

#include "InstrumentRenderModel.h"
#include "measurement/core/MeasurementAnnotation.h"
#include "measurement/core/MeasurementStore.h"
#include "measurement/core/MeasurementVisibility.h"
#include "measurement/core/Volume.h"
#include "measurement/mpr/MprResliceEngine.h"
#include "measurement/planning/InstrumentGeometry.h"
#include "measurement/vtk/VtkMprResliceAdapter.h"

#include <QImage>
#include <QPoint>
#include <QPointF>
#include <QRect>
#include <QSize>
#include <QString>
#include <QWidget>

#include <array>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

class QMouseEvent;
class QPaintEvent;
class QPainter;
class QWheelEvent;
class vtkImageData;

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

}  // namespace measurement_app
