#include "XrayDisplayWidget.h"

#include <QElapsedTimer>
#include <QEvent>
#include <QLabel>
#include <QMouseEvent>
#include <QVBoxLayout>

#include <QVTKOpenGLNativeWidget.h>

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkImageData.h>
#include <vtkImageProperty.h>
#include <vtkImageSlice.h>
#include <vtkImageSliceMapper.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLineSource.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

namespace measurement_app {
namespace {

constexpr double kXrayDrrViewportPaddingScale = 1.02;
constexpr double kXrayPi = 3.14159265358979323846;

[[nodiscard]] std::string volumeGeometrySignature(const measurement::VolumeData& volume, bool includeImageAddress)
{
    const measurement::Size3i dimensions = volume.metadata.dimensions;
    std::string signature = volume.studyUid + "|"
        + volume.seriesUid + "|"
        + volume.dataHash + "|"
        + volume.patientPositionCode + "|";
    if (includeImageAddress) {
        signature += std::to_string(reinterpret_cast<std::uintptr_t>(volume.image.get())) + "|";
    }
    signature += std::to_string(dimensions.x) + "x"
        + std::to_string(dimensions.y) + "x"
        + std::to_string(dimensions.z) + "|"
        + std::to_string(volume.metadata.spacingMm.x) + ","
        + std::to_string(volume.metadata.spacingMm.y) + ","
        + std::to_string(volume.metadata.spacingMm.z) + "|"
        + std::to_string(volume.metadata.originPatientMm.x) + ","
        + std::to_string(volume.metadata.originPatientMm.y) + ","
        + std::to_string(volume.metadata.originPatientMm.z) + "|"
        + std::to_string(volume.metadata.rowDirectionPatient.x) + ","
        + std::to_string(volume.metadata.rowDirectionPatient.y) + ","
        + std::to_string(volume.metadata.rowDirectionPatient.z) + "|"
        + std::to_string(volume.metadata.columnDirectionPatient.x) + ","
        + std::to_string(volume.metadata.columnDirectionPatient.y) + ","
        + std::to_string(volume.metadata.columnDirectionPatient.z) + "|"
        + std::to_string(volume.metadata.sliceDirectionPatient.x) + ","
        + std::to_string(volume.metadata.sliceDirectionPatient.y) + ","
        + std::to_string(volume.metadata.sliceDirectionPatient.z) + "|"
        + std::to_string(volume.transform.boundsMinPatientMm.x) + ","
        + std::to_string(volume.transform.boundsMinPatientMm.y) + ","
        + std::to_string(volume.transform.boundsMinPatientMm.z) + "|"
        + std::to_string(volume.transform.boundsMaxPatientMm.x) + ","
        + std::to_string(volume.transform.boundsMaxPatientMm.y) + ","
        + std::to_string(volume.transform.boundsMaxPatientMm.z);
    return signature;
}

[[nodiscard]] vtkSmartPointer<vtkActor> makeLineActor(
    measurement::Vec3d start,
    measurement::Vec3d end,
    const std::array<double, 3>& color,
    double lineWidth,
    double opacity)
{
    vtkNew<vtkLineSource> line;
    line->SetPoint1(start.x, start.y, start.z);
    line->SetPoint2(end.x, end.y, end.z);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(line->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(color[0], color[1], color[2]);
    actor->GetProperty()->SetLineWidth(lineWidth);
    actor->GetProperty()->SetOpacity(opacity);
    return actor;
}

[[nodiscard]] vtkSmartPointer<vtkImageData> makeVtkRgbaImage(const QImage& image)
{
    if (image.isNull() || image.width() <= 0 || image.height() <= 0) {
        return nullptr;
    }

    const QImage rgba = image.convertToFormat(QImage::Format_RGBA8888);
    vtkSmartPointer<vtkImageData> vtkImage = vtkSmartPointer<vtkImageData>::New();
    vtkImage->SetDimensions(rgba.width(), rgba.height(), 1);
    vtkImage->AllocateScalars(VTK_UNSIGNED_CHAR, 4);

    for (int y = 0; y < rgba.height(); ++y) {
        const auto* source = reinterpret_cast<const unsigned char*>(rgba.constScanLine(y));
        auto* target = static_cast<unsigned char*>(vtkImage->GetScalarPointer(0, y, 0));
        std::copy(source, source + rgba.width() * 4, target);
    }
    return vtkImage;
}

[[nodiscard]] vtkSmartPointer<vtkImageData> makeVtkFloatImage(
    int width,
    int height,
    const std::vector<float>& values)
{
    if (width <= 0 || height <= 0 || values.size() != static_cast<size_t>(width * height)) {
        return nullptr;
    }

    vtkSmartPointer<vtkImageData> vtkImage = vtkSmartPointer<vtkImageData>::New();
    vtkImage->SetDimensions(width, height, 1);
    vtkImage->AllocateScalars(VTK_FLOAT, 1);
    for (int y = 0; y < height; ++y) {
        auto* target = static_cast<float*>(vtkImage->GetScalarPointer(0, y, 0));
        std::copy(values.data() + static_cast<size_t>(y * width), values.data() + static_cast<size_t>((y + 1) * width), target);
    }
    return vtkImage;
}

}  // namespace

XrayDisplayWidget::XrayDisplayWidget(QString title, measurement::XrayPreset preset, QWidget* parent )
    : QWidget(parent)
    , m_title(std::move(title))
    , m_preset(preset)
{
    setObjectName("XrayViewport");
    setMinimumSize(260, 220);
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(3);

    m_renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    m_renderer = vtkSmartPointer<vtkRenderer>::New();
    m_renderer->SetBackground(0.06, 0.10, 0.13);
    m_renderer->SetBackground2(0.14, 0.22, 0.27);
    m_renderer->GradientBackgroundOn();
    m_renderWindow->AddRenderer(m_renderer);

    m_vtkWidget = new QVTKOpenGLNativeWidget(m_renderWindow, this);
    m_vtkWidget->setMouseTracking(true);
    m_vtkWidget->installEventFilter(this);
    layout->addWidget(m_vtkWidget, 1);

    vtkNew<vtkInteractorStyleTrackballCamera> style;
    if (m_vtkWidget->interactor() != nullptr) {
        m_vtkWidget->interactor()->SetInteractorStyle(style);
    }

    m_captionLabel = new QLabel(this);
    m_captionLabel->setObjectName("XrayCaption");
    m_captionLabel->setMinimumHeight(22);
    m_captionLabel->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    layout->addWidget(m_captionLabel);
}

void XrayDisplayWidget::setPlan(const measurement::SurgicalPlan* plan)
{
    m_plan = plan;
}

void XrayDisplayWidget::setSelectedInstrumentId(std::string id)
{
    m_selectedInstrumentId = std::move(id);
}

void XrayDisplayWidget::setVolume(const measurement::VolumeData* volume)
{
    const std::string nextSignature = volume != nullptr && volume->image
        ? volumeGeometrySignature(*volume, true)
        : std::string{};

    // DRR engines cache the uploaded CT. The VolumeData address stays the
    // same across synthetic/DICOM loads, so invalidate by voxel+geometry
    // signature before the next render.
    if (m_volume != volume || nextSignature != m_cachedVolumeSignature) {
        m_cachedVolumeSignature.clear();
        m_cudaVolumeReady = false;
        m_cpuVolumeReady = false;
        m_lineIntegral.clear();
        m_lineIntegralWidth = 0;
        m_lineIntegralHeight = 0;
        m_scalarImage = nullptr;
        m_image = {};
        m_dragMode = DragMode::None;
        m_dragInstrumentId.clear();
        m_dragTarget = DrrInteractionTarget::None;
    }
    m_volume = volume;
}

void XrayDisplayWidget::setDrrSettings(DrrUiSettings settings)
{
    m_settings = settings;
}

void XrayDisplayWidget::setPlacementActive(bool active)
{
    m_placementActive = active;
}

void XrayDisplayWidget::setPendingLine(std::optional<DrrDetectorLine> line)
{
    m_pendingLine = line;
}

void XrayDisplayWidget::setPlacementConstraints(std::array<std::optional<DrrDetectorLine>, 2> constraints)
{
    m_placementConstraints = constraints;
}

void XrayDisplayWidget::setLineCompletedCallback(std::function<void(measurement::XrayPreset, DrrDetectorLine)> callback)
{
    m_lineCompleted = std::move(callback);
}

void XrayDisplayWidget::setInstrumentSelectedCallback(std::function<void(std::string)> callback)
{
    m_instrumentSelected = std::move(callback);
}

void XrayDisplayWidget::setInstrumentDraggedCallback(
    std::function<void(measurement::XrayPreset, std::string, DrrInteractionTarget, DrrDetectorPoint)> callback)
{
    m_instrumentDragged = std::move(callback);
}

void XrayDisplayWidget::refreshOverlay()
{
    rebuildVtkScene();
}

void XrayDisplayWidget::refreshDisplaySettings()
{
    refreshDisplayMappingOnly();
}

[[nodiscard]] QImage XrayDisplayWidget::renderedImage() const
{
    return m_image;
}

void XrayDisplayWidget::refreshImage()
{
    m_image = {};
    m_lineIntegral.clear();
    m_lineIntegralWidth = 0;
    m_lineIntegralHeight = 0;
    m_scalarImage = nullptr;
    m_status = "无体数据";
    if (m_volume == nullptr || !m_volume->image) {
        m_cachedVolumeSignature.clear();
        m_cudaVolumeReady = false;
        m_cpuVolumeReady = false;
        rebuildVtkScene();
        return;
    }

    measurement::ProjectionParams projection;
    measurement::DrrRenderSettings settings;
    if (!buildRenderRequest(projection, settings)) {
        m_status = "X 光几何无效";
        rebuildVtkScene();
        return;
    }
    m_displayWindowCenter = settings.windowCenter;
    m_displayWindowWidth = settings.windowWidth;
    m_displayGamma = settings.gamma;

    m_status.clear();
    QElapsedTimer timer;
    timer.start();
    const std::string signature = currentVolumeSignature();
    if (signature != m_cachedVolumeSignature) {
        // Uploading the CT volume is the expensive CUDA step. Keep it tied to actual volume
        // identity changes so normal X-ray refreshes only submit projection
        // geometry and attenuation parameters.
        m_cudaVolumeReady = false;
        m_cpuVolumeReady = false;
        const auto cudaVolume = m_cudaEngine.setVolume(*m_volume);
        if (cudaVolume.ok()) {
            m_cudaVolumeReady = true;
        } else {
            m_status = QString("CPU 回退（%1）").arg(QString::fromStdString(cudaVolume.error().code));
        }

        const auto cpuVolume = m_cpuEngine.setVolume(*m_volume);
        if (!cpuVolume.ok()) {
            m_status = QString("%1 | %2 ms")
                           .arg(QString::fromStdString(cpuVolume.error().code))
                           .arg(timer.elapsed());
            rebuildVtkScene();
            return;
        }
        m_cpuVolumeReady = true;
        m_cachedVolumeSignature = signature;
    }

    if (m_cudaVolumeReady) {
        const auto cudaRendered = m_cudaEngine.render(projection, settings);
        if (cudaRendered.ok()) {
            cacheRenderedDrr(cudaRendered.value());
            m_status = QString("%1 | %2 ms")
                            .arg(m_image.isNull() ? "空 X 光" : "CUDA DRR")
                           .arg(timer.elapsed());
            rebuildVtkScene();
            return;
        }
        m_status = QString("CPU 回退（%1）").arg(QString::fromStdString(cudaRendered.error().code));
    }

    if (!m_cpuVolumeReady) {
        m_status = QString("CPU DRR 体数据尚未就绪 | %1 ms").arg(timer.elapsed());
        rebuildVtkScene();
        return;
    }

    const auto cpuRendered = m_cpuEngine.render(projection, settings);
    if (!cpuRendered.ok()) {
        m_status = QString("%1 | %2 ms")
                       .arg(QString::fromStdString(cpuRendered.error().code))
                       .arg(timer.elapsed());
        rebuildVtkScene();
        return;
    }

    cacheRenderedDrr(cpuRendered.value());
    if (m_status.isEmpty() || !m_status.startsWith("CPU 回退")) {
        m_status = m_image.isNull() ? "空 X 光" : "CPU DRR";
    }
    m_status = QString("%1 | %2 ms").arg(m_status).arg(timer.elapsed());
    rebuildVtkScene();
}

QSize XrayDisplayWidget::minimumSizeHint() const
{
    return {280, 240};
}

bool XrayDisplayWidget::eventFilter(QObject* watched, QEvent* event)
{
    if (watched != m_vtkWidget) {
        return QWidget::eventFilter(watched, event);
    }

    switch (event->type()) {
    case QEvent::MouseButtonPress:
        handleMousePress(static_cast<QMouseEvent*>(event));
        return true;
    case QEvent::MouseMove:
        handleMouseMove(static_cast<QMouseEvent*>(event));
        return true;
    case QEvent::MouseButtonRelease:
        handleMouseRelease(static_cast<QMouseEvent*>(event));
        return true;
    case QEvent::Wheel:
        return true;
    case QEvent::Resize:
        rebuildVtkScene();
        break;
    default:
        break;
    }
    return QWidget::eventFilter(watched, event);
}

[[nodiscard]] bool XrayDisplayWidget::buildRenderRequest(measurement::ProjectionParams& projection, measurement::DrrRenderSettings& settings) const
{
    return buildDrrRenderRequest(m_volume, m_preset, m_settings, projection, settings);
}

[[nodiscard]] bool XrayDisplayWidget::currentProjection(measurement::ProjectionParams& projection) const
{
    measurement::DrrRenderSettings settings;
    return buildRenderRequest(projection, settings);
}

bool XrayDisplayWidget::handleMousePress(QMouseEvent* event)
{
    if (event->button() != Qt::LeftButton) {
        return false;
    }

    const auto detectorPoint = widgetToDetector(event->position());
    if (!detectorPoint.has_value()) {
        return false;
    }

    if (m_placementActive) {
        m_dragMode = DragMode::Drawing;
        m_drawStart = constrainedDetectorPoint(*detectorPoint, DrrInteractionTarget::Head);
        m_drawCurrent = constrainedDetectorPoint(*detectorPoint, DrrInteractionTarget::Tail);
        rebuildVtkScene();
        return true;
    }

    const HitResult hit = hitTest(event->position());
    if (hit.target == DrrInteractionTarget::None) {
        return false;
    }

    if (m_instrumentSelected) {
        m_instrumentSelected(hit.instrumentId);
    }
    if (!hit.locked && (hit.target == DrrInteractionTarget::Head || hit.target == DrrInteractionTarget::Tail)) {
        m_dragMode = hit.target == DrrInteractionTarget::Head ? DragMode::DragHead : DragMode::DragTail;
        m_dragInstrumentId = hit.instrumentId;
        m_dragTarget = hit.target;
        if (m_instrumentDragged) {
            m_instrumentDragged(m_preset, m_dragInstrumentId, m_dragTarget, *detectorPoint);
        }
    }
    rebuildVtkScene();
    return true;
}

bool XrayDisplayWidget::handleMouseMove(QMouseEvent* event)
{
    const auto detectorPoint = widgetToDetector(event->position());
    if (!detectorPoint.has_value()) {
        return false;
    }

    switch (m_dragMode) {
    case DragMode::Drawing:
        m_drawCurrent = constrainedDetectorPoint(*detectorPoint, DrrInteractionTarget::Tail);
        rebuildVtkScene();
        return true;
    case DragMode::DragHead:
    case DragMode::DragTail:
        if (m_instrumentDragged && !m_dragInstrumentId.empty()) {
            m_instrumentDragged(m_preset, m_dragInstrumentId, m_dragTarget, *detectorPoint);
        }
        return true;
    case DragMode::None:
        updateCursorForHover(event->position());
        break;
    }
    return false;
}

bool XrayDisplayWidget::handleMouseRelease(QMouseEvent* event)
{
    if (event->button() != Qt::LeftButton || m_dragMode == DragMode::None) {
        return false;
    }

    if (m_dragMode == DragMode::Drawing) {
        const auto detectorPoint = widgetToDetector(event->position());
        if (detectorPoint.has_value()) {
            m_drawCurrent = constrainedDetectorPoint(*detectorPoint, DrrInteractionTarget::Tail);
        }
        if (std::hypot(m_drawCurrent.x - m_drawStart.x, m_drawCurrent.y - m_drawStart.y) >= 3.0 && m_lineCompleted) {
            m_lineCompleted(m_preset, {m_drawStart, m_drawCurrent});
        }
    }

    m_dragMode = DragMode::None;
    m_dragInstrumentId.clear();
    m_dragTarget = DrrInteractionTarget::None;
    updateCursorForHover(event->position());
    rebuildVtkScene();
    return true;
}

[[nodiscard]] QPointF XrayDisplayWidget::qtToVtkDisplay(QPointF widgetPoint) const
{
    if (m_vtkWidget == nullptr || m_renderWindow == nullptr) {
        return widgetPoint;
    }
    const int* renderSize = m_renderWindow->GetSize();
    const QSizeF widgetSize = m_vtkWidget->size();
    const double scaleX = widgetSize.width() > 0.0 ? static_cast<double>(std::max(renderSize[0], 1)) / widgetSize.width() : 1.0;
    const double scaleY = widgetSize.height() > 0.0 ? static_cast<double>(std::max(renderSize[1], 1)) / widgetSize.height() : 1.0;
    return {
        widgetPoint.x() * scaleX,
        (widgetSize.height() - widgetPoint.y()) * scaleY,
    };
}

[[nodiscard]] QPointF XrayDisplayWidget::vtkDisplayToQt(QPointF displayPoint) const
{
    if (m_vtkWidget == nullptr || m_renderWindow == nullptr) {
        return displayPoint;
    }
    const int* renderSize = m_renderWindow->GetSize();
    const QSizeF widgetSize = m_vtkWidget->size();
    const double scaleX = renderSize[0] > 0 ? widgetSize.width() / static_cast<double>(renderSize[0]) : 1.0;
    const double scaleY = renderSize[1] > 0 ? widgetSize.height() / static_cast<double>(renderSize[1]) : 1.0;
    return {
        displayPoint.x() * scaleX,
        widgetSize.height() - displayPoint.y() * scaleY,
    };
}

[[nodiscard]] std::optional<measurement::Vec3d> XrayDisplayWidget::displayToWorld(QPointF displayPoint, double displayZ) const
{
    if (m_renderer == nullptr) {
        return std::nullopt;
    }
    m_renderer->SetDisplayPoint(displayPoint.x(), displayPoint.y(), displayZ);
    m_renderer->DisplayToWorld();
    double world[4]{};
    m_renderer->GetWorldPoint(world);
    if (!std::isfinite(world[3]) || std::abs(world[3]) <= 1.0e-9) {
        return std::nullopt;
    }
    return measurement::Vec3d{world[0] / world[3], world[1] / world[3], world[2] / world[3]};
}

[[nodiscard]] std::optional<DrrDetectorPoint> XrayDisplayWidget::widgetToDetector(QPointF widgetPoint) const
{
    measurement::ProjectionParams projection;
    if (!currentProjection(projection) || m_renderer == nullptr) {
        return std::nullopt;
    }

    const QPointF displayPoint = qtToVtkDisplay(widgetPoint);
    const auto nearPoint = displayToWorld(displayPoint, 0.0);
    const auto farPoint = displayToWorld(displayPoint, 1.0);
    if (!nearPoint.has_value() || !farPoint.has_value()) {
        return std::nullopt;
    }

    const measurement::Vec3d rayDirection = measurement::normalize(*farPoint - *nearPoint);
    const measurement::Vec3d u = measurement::normalize(projection.detectorUPatientUnit);
    const measurement::Vec3d v = measurement::normalize(projection.detectorVPatientUnit);
    const measurement::Vec3d detectorNormal = measurement::normalize(measurement::cross(u, v));
    const double denominator = measurement::dot(rayDirection, detectorNormal);
    if (std::abs(denominator) <= 1.0e-9) {
        return std::nullopt;
    }
    const double t = measurement::dot(projection.detectorCenterPatientMm - *nearPoint, detectorNormal) / denominator;
    if (!std::isfinite(t)) {
        return std::nullopt;
    }

    const measurement::Vec3d patientPoint = *nearPoint + rayDirection * t;
    const measurement::Vec3d delta = patientPoint - projection.detectorCenterPatientMm;
    const DrrDetectorPoint detectorPoint{
        measurement::dot(delta, u) / projection.pixelSpacingMm
            + static_cast<double>(projection.detectorWidth) * 0.5
            - 0.5,
        measurement::dot(delta, v) / projection.pixelSpacingMm
            + static_cast<double>(projection.detectorHeight) * 0.5
            - 0.5,
    };
    if (detectorPoint.x < -2.0 || detectorPoint.x > static_cast<double>(projection.detectorWidth) + 1.0
        || detectorPoint.y < -2.0 || detectorPoint.y > static_cast<double>(projection.detectorHeight) + 1.0) {
        return std::nullopt;
    }
    return detectorPoint;
}

[[nodiscard]] std::optional<QPointF> XrayDisplayWidget::detectorToWidget(DrrDetectorPoint detectorPoint) const
{
    measurement::ProjectionParams projection;
    if (!currentProjection(projection) || m_renderer == nullptr) {
        return std::nullopt;
    }
    const measurement::Vec3d patientPoint = detectorPixelToPatientPoint(projection, detectorPoint, 0.0);
    m_renderer->SetWorldPoint(patientPoint.x, patientPoint.y, patientPoint.z, 1.0);
    m_renderer->WorldToDisplay();
    double display[3]{};
    m_renderer->GetDisplayPoint(display);
    if (!std::isfinite(display[0]) || !std::isfinite(display[1])) {
        return std::nullopt;
    }
    return vtkDisplayToQt({display[0], display[1]});
}

[[nodiscard]] DrrDetectorPoint XrayDisplayWidget::constrainedDetectorPoint(
    DrrDetectorPoint point,
    DrrInteractionTarget endpoint) const
{
    const size_t index = endpoint == DrrInteractionTarget::Tail ? 1U : 0U;
    if (index >= m_placementConstraints.size() || !m_placementConstraints[index].has_value()) {
        return point;
    }

    const DrrDetectorLine& line = *m_placementConstraints[index];
    const double dx = line.tail.x - line.head.x;
    const double dy = line.tail.y - line.head.y;
    const double len2 = dx * dx + dy * dy;
    if (len2 <= 1.0e-9) {
        return point;
    }
    // LAT refinement is restricted to the epipolar line produced by the AP
    // head/tail rays. The closest point gives users a forgiving draw gesture
    // while still preserving a single 3D solution.
    return closestDetectorPointOnSegment(point, line);
}

[[nodiscard]] std::vector<XrayDisplayWidget::ProjectedInstrument> XrayDisplayWidget::projectedInstruments() const
{
    std::vector<ProjectedInstrument> projected;
    measurement::ProjectionParams projection;
    if (m_plan == nullptr || !currentProjection(projection)) {
        return projected;
    }

    InstrumentRenderModelBuilder builder;
    for (const InstrumentRenderModel& model : builder.buildVisibleModels(*m_plan, m_selectedInstrumentId)) {
        for (const InstrumentRenderSegment* segment : {&model.headSegment, &model.tailSegment}) {
            if (segment->lengthMm <= 1.0e-6) {
                continue;
            }
            const auto head = projectPatientToDetectorPixel(projection, segment->startPatientMm);
            const auto tail = projectPatientToDetectorPixel(projection, segment->endPatientMm);
            if (!head.has_value() || !tail.has_value()) {
                continue;
            }
            projected.push_back({
                segment->instrumentId,
                segment->locked,
                segment->role,
                segment->style,
                *head,
                *tail,
            });
        }
    }
    return projected;
}

[[nodiscard]] XrayDisplayWidget::HitResult XrayDisplayWidget::hitTest(QPointF widgetPoint) const
{
    constexpr double kSegmentHitDistancePx = 8.0;

    const std::vector<ProjectedInstrument> instruments = projectedInstruments();
    HitResult best;
    double bestDistance = std::numeric_limits<double>::max();

    for (const ProjectedInstrument& instrument : instruments) {
        const auto head = detectorToWidget(instrument.head);
        const auto tail = detectorToWidget(instrument.tail);
        if (!head.has_value() || !tail.has_value()) {
            continue;
        }
        const double distance = distancePointToSegmentPx(
            {widgetPoint.x(), widgetPoint.y()},
            {head->x(), head->y()},
            {tail->x(), tail->y()});
        if (distance <= kSegmentHitDistancePx && distance < bestDistance) {
            best = {
                instrument.role == InstrumentRenderSegmentRole::Head
                    ? DrrInteractionTarget::Head
                    : DrrInteractionTarget::Tail,
                instrument.id,
                instrument.locked};
            bestDistance = distance;
        }
    }
    return best;
}

void XrayDisplayWidget::updateCursorForHover(QPoint position)
{
    updateCursorForHover(QPointF(position));
}

void XrayDisplayWidget::updateCursorForHover(QPointF position)
{
    if (m_placementActive) {
        m_vtkWidget->setCursor(Qt::CrossCursor);
        return;
    }
    const HitResult hit = hitTest(position);
    if (hit.target == DrrInteractionTarget::Head || hit.target == DrrInteractionTarget::Tail) {
        m_vtkWidget->setCursor(hit.locked ? Qt::ForbiddenCursor : Qt::SizeAllCursor);
    } else if (hit.target == DrrInteractionTarget::Body) {
        m_vtkWidget->setCursor(Qt::PointingHandCursor);
    } else {
        m_vtkWidget->unsetCursor();
    }
}

[[nodiscard]] measurement::Vec3d XrayDisplayWidget::detectorPixelToPatientPoint(
    const measurement::ProjectionParams& projection,
    DrrDetectorPoint point,
    double planeOffsetMm ) const
{
    const measurement::Vec3d u = measurement::normalize(projection.detectorUPatientUnit);
    const measurement::Vec3d v = measurement::normalize(projection.detectorVPatientUnit);
    const measurement::Vec3d toSource = measurement::normalize(projection.sourcePosPatientMm - projection.detectorCenterPatientMm);
    const double centeredX = point.x + 0.5 - static_cast<double>(projection.detectorWidth) * 0.5;
    const double centeredY = point.y + 0.5 - static_cast<double>(projection.detectorHeight) * 0.5;
    return projection.detectorCenterPatientMm
        + u * (centeredX * projection.pixelSpacingMm)
        + v * (centeredY * projection.pixelSpacingMm)
        + toSource * planeOffsetMm;
}

[[nodiscard]] vtkSmartPointer<vtkActor> XrayDisplayWidget::makeDrrHandleActor(
    measurement::Vec3d center,
    const std::array<double, 3>& color,
    double radiusMm,
    double opacity) const
{
    vtkNew<vtkSphereSource> sphere;
    sphere->SetCenter(center.x, center.y, center.z);
    sphere->SetRadius(radiusMm);
    sphere->SetThetaResolution(18);
    sphere->SetPhiResolution(18);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(sphere->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(color[0], color[1], color[2]);
    actor->GetProperty()->SetOpacity(opacity);
    actor->GetProperty()->LightingOff();
    return actor;
}

void XrayDisplayWidget::addDetectorLineActor(
    const measurement::ProjectionParams& projection,
    const DrrDetectorLine& line,
    const std::array<double, 3>& color,
    double lineWidth,
    double opacity,
    bool handles)
{
    if (m_renderer == nullptr) {
        return;
    }
    const double overlayOffsetMm = std::max(0.5, projection.pixelSpacingMm * 0.75);
    const measurement::Vec3d head = detectorPixelToPatientPoint(projection, line.head, overlayOffsetMm);
    const measurement::Vec3d tail = detectorPixelToPatientPoint(projection, line.tail, overlayOffsetMm);
    m_renderer->AddActor(makeLineActor(head, tail, color, lineWidth, opacity));
    if (handles) {
        const double handleRadius = std::clamp(projection.pixelSpacingMm * 4.0, 1.5, 8.0);
        m_renderer->AddActor(makeDrrHandleActor(head, color, handleRadius, opacity));
        m_renderer->AddActor(makeDrrHandleActor(tail, color, handleRadius, opacity));
    }
}

[[nodiscard]] vtkSmartPointer<vtkMatrix4x4> XrayDisplayWidget::makeImageSliceMatrix(
    const measurement::ProjectionParams& projection) const
{
    vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
    matrix->Identity();

    const measurement::Vec3d u = measurement::normalize(projection.detectorUPatientUnit);
    const measurement::Vec3d v = measurement::normalize(projection.detectorVPatientUnit);
    const measurement::Vec3d n = measurement::normalize(measurement::cross(u, v));
    const measurement::Vec3d origin = projection.detectorCenterPatientMm
        - u * (0.5 * static_cast<double>(std::max(1, projection.detectorWidth - 1)) * projection.pixelSpacingMm)
        - v * (0.5 * static_cast<double>(std::max(1, projection.detectorHeight - 1)) * projection.pixelSpacingMm);

    // The vtkImageData stays in pixel index space; this user matrix maps one
    // image pixel to one detector pixel in patient millimeters.
    matrix->SetElement(0, 0, u.x * projection.pixelSpacingMm);
    matrix->SetElement(1, 0, u.y * projection.pixelSpacingMm);
    matrix->SetElement(2, 0, u.z * projection.pixelSpacingMm);
    matrix->SetElement(0, 1, v.x * projection.pixelSpacingMm);
    matrix->SetElement(1, 1, v.y * projection.pixelSpacingMm);
    matrix->SetElement(2, 1, v.z * projection.pixelSpacingMm);
    matrix->SetElement(0, 2, n.x);
    matrix->SetElement(1, 2, n.y);
    matrix->SetElement(2, 2, n.z);
    matrix->SetElement(0, 3, origin.x);
    matrix->SetElement(1, 3, origin.y);
    matrix->SetElement(2, 3, origin.z);
    return matrix;
}

void XrayDisplayWidget::configureCamera(const measurement::ProjectionParams& projection)
{
    if (m_renderer == nullptr || m_vtkWidget == nullptr) {
        return;
    }

    vtkCamera* camera = m_renderer->GetActiveCamera();
    if (camera == nullptr) {
        return;
    }

    const measurement::Vec3d source = projection.sourcePosPatientMm;
    const measurement::Vec3d target = projection.detectorCenterPatientMm;
    const measurement::Vec3d viewUp = measurement::normalize(projection.detectorVPatientUnit);
    camera->SetPosition(source.x, source.y, source.z);
    camera->SetFocalPoint(target.x, target.y, target.z);
    camera->SetViewUp(viewUp.x, viewUp.y, viewUp.z);
    camera->SetParallelProjection(false);

    const double detectorWidthMm = std::max(1.0, static_cast<double>(std::max(1, projection.detectorWidth)) * projection.pixelSpacingMm);
    const double detectorHeightMm = std::max(1.0, static_cast<double>(std::max(1, projection.detectorHeight)) * projection.pixelSpacingMm);
    const QSize viewportSize = m_vtkWidget->size();
    const double viewportAspect = viewportSize.height() > 0
        ? static_cast<double>(std::max(1, viewportSize.width())) / static_cast<double>(viewportSize.height())
        : detectorWidthMm / detectorHeightMm;
    const double neededHeightMm = std::max(detectorHeightMm, detectorWidthMm / std::max(viewportAspect, 1.0e-3));
    const double distanceMm = std::max(1.0, measurement::length(target - source));
    const double viewAngleDeg = 2.0 * std::atan((neededHeightMm * 0.5 * kXrayDrrViewportPaddingScale) / distanceMm) * 180.0 / kXrayPi;
    camera->SetViewAngle(std::clamp(viewAngleDeg, 1.0, 120.0));
    camera->SetClippingRange(1.0, distanceMm + std::max(2000.0, distanceMm * 2.0));
    m_renderer->ResetCameraClippingRange();
}

void XrayDisplayWidget::rebuildVtkScene()
{
    if (m_renderer == nullptr || m_renderWindow == nullptr) {
        return;
    }

    m_renderer->RemoveAllViewProps();
    measurement::ProjectionParams projection;
    const bool hasProjection = currentProjection(projection);
    if (hasProjection && (m_scalarImage != nullptr || !m_image.isNull())) {
        vtkSmartPointer<vtkImageData> vtkImage = m_scalarImage != nullptr ? m_scalarImage : makeVtkRgbaImage(m_image);
        if (vtkImage != nullptr) {
            vtkNew<vtkImageSliceMapper> mapper;
            mapper->SetInputData(vtkImage);

            vtkNew<vtkImageSlice> imageSlice;
            imageSlice->SetMapper(mapper);
            imageSlice->SetUserMatrix(makeImageSliceMatrix(projection));
            imageSlice->GetProperty()->SetInterpolationTypeToLinear();
            imageSlice->GetProperty()->SetOpacity(1.0);
            if (m_scalarImage != nullptr) {
                imageSlice->GetProperty()->SetColorWindow(std::max(m_displayWindowWidth, 1.0));
                imageSlice->GetProperty()->SetColorLevel(m_displayWindowCenter);
            }
            m_renderer->AddViewProp(imageSlice);
        }
    }

    if (hasProjection) {
        for (const ProjectedInstrument& instrument : projectedInstruments()) {
            addDetectorLineActor(
                projection,
                {instrument.head, instrument.tail},
                instrument.style.color,
                instrument.style.lineWidth,
                instrument.style.opacity,
                false);
        }
        if (m_pendingLine.has_value()) {
            addDetectorLineActor(projection, *m_pendingLine, {1.0, 0.70, 0.24}, 2.4, 0.88, true);
        }
        for (const std::optional<DrrDetectorLine>& constraint : m_placementConstraints) {
            if (constraint.has_value()) {
                addDetectorLineActor(projection, *constraint, {0.95, 0.45, 1.0}, 1.8, 0.62, false);
            }
        }
        if (m_dragMode == DragMode::Drawing) {
            addDetectorLineActor(projection, {m_drawStart, m_drawCurrent}, {0.0, 0.96, 0.74}, 2.4, 0.92, true);
        }
        configureCamera(projection);
    }

    if (m_captionLabel != nullptr) {
        const QString caption = m_status.isEmpty() ? m_title : QString("%1 | %2").arg(m_title, m_status);
        m_captionLabel->setText(caption);
    }
    m_renderWindow->Render();
}

[[nodiscard]] std::string XrayDisplayWidget::currentVolumeSignature() const
{
    if (m_volume == nullptr || !m_volume->image) {
        return {};
    }
    // The app owns one VolumeData object whose address stays stable while loads replace its
    // contents, so the signature must describe both the voxel payload and the current patient
    // geometry. Manual patient-position overrides change the geometry without changing voxels.
    return volumeGeometrySignature(*m_volume, true);
}

[[nodiscard]] QImage XrayDisplayWidget::imageFromLineIntegral(int width, int height, const std::vector<float>& lineIntegral) const
{
    if (width <= 0 || height <= 0 || lineIntegral.size() != static_cast<size_t>(width * height)) {
        return {};
    }

    const double windowWidth = std::max(m_displayWindowWidth, 1.0);
    const double lower = m_displayWindowCenter - windowWidth * 0.5;
    const double inverseGamma = 1.0 / std::max(m_displayGamma, 1.0e-6);
    QImage image(width, height, QImage::Format_RGB32);
    for (int y = 0; y < height; ++y) {
        auto* scanline = reinterpret_cast<QRgb*>(image.scanLine(y));
        for (int x = 0; x < width; ++x) {
            const double integral = static_cast<double>(lineIntegral[static_cast<size_t>(y * width + x)]);
            const double normalized = std::clamp((integral - lower) / windowWidth, 0.0, 1.0);
            const int gray = static_cast<int>(std::clamp(std::pow(normalized, inverseGamma), 0.0, 1.0) * 255.0);
            scanline[x] = qRgb(gray, gray, gray);
        }
    }
    return image;
}

[[nodiscard]] QImage XrayDisplayWidget::imageFromDrr(const measurement::DrrImage& drr) const
{
    if (drr.width <= 0 || drr.height <= 0 || drr.displayImage.size() != static_cast<size_t>(drr.width * drr.height)) {
        return {};
    }

    QImage image(drr.width, drr.height, QImage::Format_RGB32);
    for (int y = 0; y < drr.height; ++y) {
        auto* scanline = reinterpret_cast<QRgb*>(image.scanLine(y));
        for (int x = 0; x < drr.width; ++x) {
            const uint16_t value = drr.displayImage[static_cast<size_t>(y * drr.width + x)];
            const int gray = static_cast<int>(value / 257U);
            scanline[x] = qRgb(gray, gray, gray);
        }
    }
    return image;
}

void XrayDisplayWidget::cacheRenderedDrr(const measurement::DrrImage& drr)
{
    if (drr.width > 0 && drr.height > 0 && drr.lineIntegral.size() == static_cast<size_t>(drr.width * drr.height)) {
        m_lineIntegral = drr.lineIntegral;
        m_lineIntegralWidth = drr.width;
        m_lineIntegralHeight = drr.height;
        m_scalarImage = makeVtkFloatImage(drr.width, drr.height, m_lineIntegral);
        m_image = imageFromLineIntegral(drr.width, drr.height, m_lineIntegral);
        return;
    }
    m_lineIntegral.clear();
    m_lineIntegralWidth = 0;
    m_lineIntegralHeight = 0;
    m_scalarImage = nullptr;
    m_image = imageFromDrr(drr);
}

void XrayDisplayWidget::refreshDisplayMappingOnly()
{
    measurement::ProjectionParams projection;
    measurement::DrrRenderSettings settings;
    if (buildRenderRequest(projection, settings)) {
        m_displayWindowCenter = settings.windowCenter;
        m_displayWindowWidth = settings.windowWidth;
        m_displayGamma = settings.gamma;
    }
    if (m_lineIntegralWidth > 0 && m_lineIntegralHeight > 0 && !m_lineIntegral.empty()) {
        m_image = imageFromLineIntegral(m_lineIntegralWidth, m_lineIntegralHeight, m_lineIntegral);
    }
    rebuildVtkScene();
}

}  // namespace measurement_app
