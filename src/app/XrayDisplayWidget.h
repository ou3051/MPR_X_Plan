#pragma once

#include "DrrInteractionGeometry.h"
#include "DrrRenderRequest.h"
#include "InstrumentRenderModel.h"
#include "measurement/core/Geometry.h"
#include "measurement/core/Instrument.h"
#include "measurement/core/Volume.h"
#include "measurement/core/Xray.h"
#include "measurement/drr/CpuDrrEngine.h"
#include "measurement/drr/CudaDrrEngine.h"

#include <QImage>
#include <QPointF>
#include <QSize>
#include <QString>
#include <QWidget>

#include <vtkSmartPointer.h>

#include <array>
#include <functional>
#include <optional>
#include <string>
#include <vector>

class QLabel;
class QEvent;
class QMouseEvent;
class QVTKOpenGLNativeWidget;
class vtkActor;
class vtkGenericOpenGLRenderWindow;
class vtkImageData;
class vtkMatrix4x4;
class vtkRenderer;

namespace measurement_app {

class XrayDisplayWidget final : public QWidget {
public:
    XrayDisplayWidget(QString title, measurement::XrayPreset preset, QWidget* parent = nullptr);

    void setPlan(const measurement::SurgicalPlan* plan);

    void setSelectedInstrumentId(std::string id);

    void setVolume(const measurement::VolumeData* volume);

    void setDrrSettings(DrrUiSettings settings);

    void setPlacementActive(bool active);

    void setPendingLine(std::optional<DrrDetectorLine> line);

    void setPlacementConstraints(std::array<std::optional<DrrDetectorLine>, 2> constraints);

    void setLineCompletedCallback(std::function<void(measurement::XrayPreset, DrrDetectorLine)> callback);

    void setInstrumentSelectedCallback(std::function<void(std::string)> callback);

    void setInstrumentDraggedCallback(
        std::function<void(measurement::XrayPreset, std::string, DrrInteractionTarget, DrrDetectorPoint)> callback);

    void refreshOverlay();

    void refreshDisplaySettings();

    [[nodiscard]] QImage renderedImage() const;

    void refreshImage();

protected:
    QSize minimumSizeHint() const override;

    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    enum class DragMode {
        None,
        Drawing,
        DragHead,
        DragTail
    };

    struct ProjectedInstrument {
        std::string id;
        bool locked = false;
        InstrumentRenderSegmentRole role = InstrumentRenderSegmentRole::Head;
        InstrumentRenderStyle style;
        DrrDetectorPoint head;
        DrrDetectorPoint tail;
    };

    struct HitResult {
        DrrInteractionTarget target = DrrInteractionTarget::None;
        std::string instrumentId;
        bool locked = false;
    };

    [[nodiscard]] bool buildRenderRequest(measurement::ProjectionParams& projection, measurement::DrrRenderSettings& settings) const;

    [[nodiscard]] bool currentProjection(measurement::ProjectionParams& projection) const;

    bool handleMousePress(QMouseEvent* event);

    bool handleMouseMove(QMouseEvent* event);

    bool handleMouseRelease(QMouseEvent* event);

    [[nodiscard]] QPointF qtToVtkDisplay(QPointF widgetPoint) const;

    [[nodiscard]] QPointF vtkDisplayToQt(QPointF displayPoint) const;

    [[nodiscard]] std::optional<measurement::Vec3d> displayToWorld(QPointF displayPoint, double displayZ) const;

    [[nodiscard]] std::optional<DrrDetectorPoint> widgetToDetector(QPointF widgetPoint) const;

    [[nodiscard]] std::optional<QPointF> detectorToWidget(DrrDetectorPoint detectorPoint) const;

    [[nodiscard]] DrrDetectorPoint constrainedDetectorPoint(
        DrrDetectorPoint point,
        DrrInteractionTarget endpoint) const;

    [[nodiscard]] std::vector<ProjectedInstrument> projectedInstruments() const;

    [[nodiscard]] HitResult hitTest(QPointF widgetPoint) const;

    void updateCursorForHover(QPoint position);

    void updateCursorForHover(QPointF position);

    [[nodiscard]] measurement::Vec3d detectorPixelToPatientPoint(
        const measurement::ProjectionParams& projection,
        DrrDetectorPoint point,
        double planeOffsetMm = 0.0) const;

    [[nodiscard]] vtkSmartPointer<vtkActor> makeDrrHandleActor(
        measurement::Vec3d center,
        const std::array<double, 3>& color,
        double radiusMm,
        double opacity) const;

    void addDetectorLineActor(
        const measurement::ProjectionParams& projection,
        const DrrDetectorLine& line,
        const std::array<double, 3>& color,
        double lineWidth,
        double opacity,
        bool handles);

    [[nodiscard]] vtkSmartPointer<vtkMatrix4x4> makeImageSliceMatrix(
        const measurement::ProjectionParams& projection) const;

    void configureCamera(const measurement::ProjectionParams& projection);

    void rebuildVtkScene();

    [[nodiscard]] std::string currentVolumeSignature() const;

    [[nodiscard]] QImage imageFromLineIntegral(int width, int height, const std::vector<float>& lineIntegral) const;

    [[nodiscard]] QImage imageFromDrr(const measurement::DrrImage& drr) const;

    void cacheRenderedDrr(const measurement::DrrImage& drr);

    void refreshDisplayMappingOnly();

    QString m_title;
    measurement::XrayPreset m_preset = measurement::XrayPreset::AP;
    const measurement::VolumeData* m_volume = nullptr;
    const measurement::SurgicalPlan* m_plan = nullptr;
    DrrUiSettings m_settings;
    measurement::CudaDrrEngine m_cudaEngine;
    measurement::CpuDrrEngine m_cpuEngine;
    std::string m_cachedVolumeSignature;
    std::string m_selectedInstrumentId;
    QImage m_image;
    std::vector<float> m_lineIntegral;
    int m_lineIntegralWidth = 0;
    int m_lineIntegralHeight = 0;
    vtkSmartPointer<vtkImageData> m_scalarImage;
    double m_displayWindowCenter = 0.0;
    double m_displayWindowWidth = 1.0;
    double m_displayGamma = 1.0;
    QString m_status = "No volume";
    bool m_placementActive = false;
    std::optional<DrrDetectorLine> m_pendingLine;
    std::array<std::optional<DrrDetectorLine>, 2> m_placementConstraints{};
    DragMode m_dragMode = DragMode::None;
    DrrDetectorPoint m_drawStart;
    DrrDetectorPoint m_drawCurrent;
    std::string m_dragInstrumentId;
    DrrInteractionTarget m_dragTarget = DrrInteractionTarget::None;
    std::function<void(measurement::XrayPreset, DrrDetectorLine)> m_lineCompleted;
    std::function<void(std::string)> m_instrumentSelected;
    std::function<void(measurement::XrayPreset, std::string, DrrInteractionTarget, DrrDetectorPoint)> m_instrumentDragged;
    QVTKOpenGLNativeWidget* m_vtkWidget = nullptr;
    QLabel* m_captionLabel = nullptr;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderWindow;
    vtkSmartPointer<vtkRenderer> m_renderer;
    bool m_cudaVolumeReady = false;
    bool m_cpuVolumeReady = false;
};

}  // namespace measurement_app
