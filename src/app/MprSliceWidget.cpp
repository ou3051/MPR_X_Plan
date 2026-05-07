#include "MprSliceWidget.h"

#include "InstrumentRenderModel.h"
#include "measurement/core/MeasurementVisibility.h"

#include <QApplication>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QWheelEvent>

#include <vtkImageData.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace measurement_app {
namespace {
constexpr int kSlicePixels = 320;
constexpr double kDefaultPixelSpacingMm = 1.0;
constexpr double kCrosshairHitTolerancePx = 8.0;
constexpr double kCrosshairCenterRadiusPx = 10.0;
constexpr double kMinZoom = 0.25;
constexpr double kMaxZoom = 16.0;
constexpr double kZoomDragSensitivity = 0.01;
constexpr double kWindowCenterSensitivityHuPerPixel = 4.0;
constexpr double kWindowWidthSensitivityHuPerPixel = 8.0;
constexpr double kRotationHandleDistancePx = 40.0;
constexpr double kRotationHandleRadiusPx = 8.0;
constexpr double kRotationHandleMarginPx = 16.0;
constexpr double kRotationHandleInsetPx = 30.0;
constexpr double kRotationHandleBarLengthPx = 10.0;
constexpr double kRotationHandleBarGapPx = 5.0;
constexpr double kOrientationLabelMarginPx = 10.0;
constexpr double kOrientationLabelInsetPx = 14.0;
constexpr double kPi = 3.14159265358979323846;

[[nodiscard]] const char* planeTitle(measurement::MprPlane plane)
{
    switch (plane) {
    case measurement::MprPlane::Axial:
        return "轴位";
    case measurement::MprPlane::Sagittal:
        return "矢状位";
    case measurement::MprPlane::Coronal:
        return "冠状位";
    }
    return "MPR";
}

[[nodiscard]] double clampDouble(double value, double minValue, double maxValue)
{
    return std::max(minValue, std::min(maxValue, value));
}
[[nodiscard]] bool isFiniteVec(measurement::Vec3d value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

[[nodiscard]] double clampZoom(double zoom)
{
    return clampDouble(zoom, kMinZoom, kMaxZoom);
}

[[nodiscard]] int planeIndex(measurement::MprPlane plane)
{
    switch (plane) {
    case measurement::MprPlane::Axial:
        return 0;
    case measurement::MprPlane::Sagittal:
        return 1;
    case measurement::MprPlane::Coronal:
        return 2;
    }
    return 0;
}

[[nodiscard]] measurement::Vec3d stableCrosslineDirectionPatient(
    const measurement::VolumeData* volume,
    const std::array<measurement::MprSliceFrame, 3>& frames,
    measurement::MprPlane firstPlane,
    measurement::MprPlane secondPlane)
{
    if (firstPlane == secondPlane) {
        return {};
    }

    measurement::MprPlane lowerPlane = firstPlane;
    measurement::MprPlane upperPlane = secondPlane;
    if (planeIndex(lowerPlane) > planeIndex(upperPlane)) {
        std::swap(lowerPlane, upperPlane);
    }

    measurement::Vec3d direction = measurement::normalize(measurement::cross(
        frames[planeIndex(lowerPlane)].normalPatientUnit,
        frames[planeIndex(upperPlane)].normalPatientUnit));
    if (!isFiniteVec(direction) || measurement::length(direction) <= 1.0e-6) {
        return {};
    }

    if (volume != nullptr) {
        const measurement::Vec3d reference = measurement::normalize(measurement::cross(
            measurement::planeNormalPatient(volume->metadata, lowerPlane),
            measurement::planeNormalPatient(volume->metadata, upperPlane)));
        if (isFiniteVec(reference)
            && measurement::length(reference) > 1.0e-6
            && measurement::dot(direction, reference) < 0.0) {
            direction = direction * -1.0;
        }
    }

    return direction;
}

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

[[nodiscard]] QString orientationLabelForPatientVector(measurement::Vec3d directionPatient)
{
    const measurement::Vec3d unit = measurement::normalize(directionPatient);
    if (!isFiniteVec(unit) || measurement::length(unit) <= 1.0e-6) {
        return {};
    }

    struct AxisComponent {
        double magnitude = 0.0;
        QChar label;
    };

    std::array<AxisComponent, 3> components{
        AxisComponent{std::abs(unit.x), unit.x >= 0.0 ? QChar('L') : QChar('R')},
        AxisComponent{std::abs(unit.y), unit.y >= 0.0 ? QChar('P') : QChar('A')},
        AxisComponent{std::abs(unit.z), unit.z >= 0.0 ? QChar('H') : QChar('F')},
    };
    std::sort(
        components.begin(),
        components.end(),
        [](const AxisComponent& lhs, const AxisComponent& rhs) { return lhs.magnitude > rhs.magnitude; });

    QString label;
    if (components[0].magnitude >= 1.0e-3) {
        label.append(components[0].label);
    }
    if (components[1].magnitude >= 0.35 && components[1].magnitude >= components[0].magnitude * 0.45) {
        label.append(components[1].label);
    }
    return label;
}

}  // namespace

MprSliceWidget::MprSliceWidget(measurement::MprPlane plane, QWidget* parent)
    : QWidget(parent)
    , m_plane(plane)
{
    m_request.outputWidth = kSlicePixels;
    m_request.outputHeight = kSlicePixels;
    m_request.pixelSpacingMm = kDefaultPixelSpacingMm;
    setMouseTracking(true);
    setMinimumSize(240, 240);
}

void MprSliceWidget::setVolume(const measurement::VolumeData* volume)
{
    const std::string nextSignature = volume != nullptr && volume->image
        ? volumeGeometrySignature(*volume, true)
        : std::string{};

    // The main window owns one stable VolumeData object. Loading DICOM replaces
    // its contents in-place, so pointer comparison alone cannot tell the slice
    // widgets that synthetic voxels/geometries have been replaced.
    if (m_volume != volume || m_volumeSignature != nextSignature) {
        m_pan = {};
        m_zoom = 1.0;
        m_dragMode = DragMode::None;
        m_lastRotationAngleRad = 0.0;
        m_resliceAdapter = measurement::VtkMprResliceAdapter();
    }
    m_volume = volume;
    m_volumeSignature = nextSignature;
}

measurement::MprPlane MprSliceWidget::plane() const
{
    return m_plane;
}

void MprSliceWidget::setState(const measurement::MprViewState* state)
{
    m_state = state;
    if (m_state != nullptr && m_dragMode != DragMode::WindowLevel) {
        m_windowCenterHu = m_state->windowCenterHu;
        m_windowWidthHu = std::max(m_state->windowWidthHu, 1.0);
    }
}

void MprSliceWidget::setLinkedPlaneFrames(const std::array<measurement::MprSliceFrame, 3>* frames)
{
    m_linkedPlaneFrames = frames;
}

void MprSliceWidget::setPlan(const measurement::SurgicalPlan* plan)
{
    m_plan = plan;
}

void MprSliceWidget::setRequest(measurement::MprSliceRequest request)
{
    m_request = request;
}

void MprSliceWidget::setSelectedInstrumentId(std::string id)
{
    m_selectedInstrumentId = std::move(id);
    update();
}

void MprSliceWidget::setCrosshairChangedCallback(std::function<void(measurement::Vec3d)> callback)
{
    m_crosshairChanged = std::move(callback);
}

void MprSliceWidget::setWindowLevelChangedCallback(std::function<void(double, double)> callback)
{
    m_windowLevelChanged = std::move(callback);
}

void MprSliceWidget::setPlaneRotationCallback(std::function<void(measurement::MprPlane, measurement::MprPlane, double)> callback)
{
    m_planeRotationChanged = std::move(callback);
}

void MprSliceWidget::setActivatedCallback(std::function<void(measurement::MprPlane)> callback)
{
    m_activated = std::move(callback);
}

void MprSliceWidget::resetViewPresentation()
{
    m_pan = {};
    m_zoom = 1.0;
    if (m_state != nullptr) {
        m_windowCenterHu = m_state->windowCenterHu;
        m_windowWidthHu = std::max(m_state->windowWidthHu, 1.0);
    }
    refreshImage();
}

QSize MprSliceWidget::minimumSizeHint() const
{
    return {260, 260};
}

measurement::Result<measurement::MprResliceParameters> MprSliceWidget::parameters() const
{
    if (m_volume == nullptr || m_state == nullptr) {
        return measurement::Result<measurement::MprResliceParameters>::failure(
        measurement::makeErrorInfo("MPR_VIEW_NOT_READY", "MPR 视图尚未就绪。"));
    }
    return measurement::buildMprResliceParameters(*m_volume, stateForPlane(), m_request);
}

measurement::MprViewState MprSliceWidget::stateForPlane() const
{
    measurement::MprViewState state;
    if (m_state != nullptr) {
        state = *m_state;
    }
    state.plane = m_plane;
    state.zoom = m_zoom;
    state.pan = m_pan;
    state.windowCenterHu = m_windowCenterHu;
    state.windowWidthHu = m_windowWidthHu;
    return state;
}

QImage MprSliceWidget::imageFromVtkReslice(vtkImageData& image) const
{
    int dimensions[3] = {0, 0, 0};
    image.GetDimensions(dimensions);
    if (dimensions[0] <= 0 || dimensions[1] <= 0) {
        return {};
    }

    const double center = m_state != nullptr ? m_state->windowCenterHu : 400.0;
    const double width = std::max(m_state != nullptr ? m_state->windowWidthHu : 2000.0, 1.0);
    const double lower = center - width * 0.5;
    const int16_t backgroundHu = m_volume != nullptr ? static_cast<int16_t>(m_volume->metadata.minHu) : static_cast<int16_t>(-1000);

    QImage rendered(dimensions[0], dimensions[1], QImage::Format_RGB32);
    for (int y = 0; y < dimensions[1]; ++y) {
        auto* scanline = reinterpret_cast<QRgb*>(rendered.scanLine(y));
        const int vtkY = dimensions[1] - 1 - y;
        for (int x = 0; x < dimensions[0]; ++x) {
            auto* scalar = static_cast<int16_t*>(image.GetScalarPointer(x, vtkY, 0));
            const int16_t hu = scalar != nullptr ? *scalar : backgroundHu;
            const double normalized = clampDouble((static_cast<double>(hu) - lower) / width, 0.0, 1.0);
            const int gray = static_cast<int>(std::llround(normalized * 255.0));
            scanline[x] = qRgb(gray, gray, gray);
        }
    }
    return rendered;
}

void MprSliceWidget::refreshImage()
{
    const auto params = parameters();
    if (!params.ok() || m_volume == nullptr) {
        m_instrumentSections.clear();
        m_image = QImage();
        m_renderStatus = params.ok() ? "MPR 体数据不可用。" : QString::fromStdString(params.error().code + ": " + params.error().detail);
        update();
        return;
    }

    const auto reslice = m_resliceAdapter.reslice(*m_volume, stateForPlane(), m_request);
    if (!reslice.ok() || reslice.value().image == nullptr || !reslice.value().readyToRender) {
        m_instrumentSections.clear();
        m_image = QImage();
        m_renderStatus = reslice.ok()
            ? "MPR 适配器未返回可渲染图像。"
            : QString::fromStdString(reslice.error().code + ": " + reslice.error().detail);
        update();
        return;
    }

    m_image = imageFromVtkReslice(*reslice.value().image);
    if (m_image.isNull()) {
        m_renderStatus = "MPR 适配器返回了空 vtkImageData。";
    } else {
        m_renderStatus = "vtkImageReslice";
    }

    if (m_plan != nullptr) {
        InstrumentRenderModelBuilder builder;
        m_instrumentSections = builder.buildVisibleSectionSegments(
            *m_plan,
            {params.value().frame.originPatientMm, params.value().frame.normalPatientUnit},
            m_selectedInstrumentId);
    } else {
        m_instrumentSections.clear();
    }
    update();
}

QRect MprSliceWidget::imageRect() const
{
    return rect().adjusted(8, 28, -8, -8);
}

QPointF MprSliceWidget::clampImagePoint(QPointF imagePoint) const
{
    return {
        clampDouble(imagePoint.x(), 0.0, static_cast<double>(std::max(m_image.width() - 1, 0))),
        clampDouble(imagePoint.y(), 0.0, static_cast<double>(std::max(m_image.height() - 1, 0))),
    };
}

QPointF MprSliceWidget::imageDirectionForPlane(
    const measurement::MprSliceFrame& currentFrame,
    measurement::Vec3d otherPlaneNormal) const
{
    const measurement::Vec3d directionPatient = measurement::normalize(
        measurement::cross(currentFrame.normalPatientUnit, otherPlaneNormal));
    if (!isFiniteVec(directionPatient) || measurement::length(directionPatient) <= 0.0) {
        return {};
    }

    return {
        measurement::dot(directionPatient, currentFrame.horizontalPatientUnit),
        -measurement::dot(directionPatient, currentFrame.verticalPatientUnit),
    };
}

std::array<MprSliceWidget::CrosslineInfo, 2> MprSliceWidget::crosslines() const
{
    std::array<CrosslineInfo, 2> lines{};
    if (m_state == nullptr || m_linkedPlaneFrames == nullptr) {
        return lines;
    }

    const auto params = parameters();
    if (!params.ok()) {
        return lines;
    }

    const int currentIndex = planeIndex(m_plane);
    const measurement::MprSliceFrame& currentFrame = params.value().frame;
    const QPointF center = patientToImagePoint(m_state->crosshairPatientMm);
    int lineSlot = 0;
    for (int index = 0; index < static_cast<int>(m_linkedPlaneFrames->size()); ++index) {
        if (index == currentIndex) {
            continue;
        }

        measurement::Vec3d directionPatient = measurement::normalize(
            measurement::cross(currentFrame.normalPatientUnit, (*m_linkedPlaneFrames)[index].normalPatientUnit));
        const QPointF direction = imageDirectionForPlane(currentFrame, (*m_linkedPlaneFrames)[index].normalPatientUnit);
        const double directionLength = std::hypot(direction.x(), direction.y());
        if (directionLength <= 1.0e-6) {
            continue;
        }

        QPointF stableDirection = direction;
        const measurement::MprPlane sourcePlane = static_cast<measurement::MprPlane>(index);
        const measurement::Vec3d stableDirectionPatient =
            stableCrosslineDirectionPatient(m_volume, *m_linkedPlaneFrames, m_plane, sourcePlane);
        if (isFiniteVec(stableDirectionPatient)
            && measurement::length(stableDirectionPatient) > 1.0e-6
            && measurement::dot(directionPatient, stableDirectionPatient) < 0.0) {
            directionPatient = directionPatient * -1.0;
            stableDirection = {-direction.x(), -direction.y()};
        }

        lines[lineSlot].sourcePlane = static_cast<measurement::MprPlane>(index);
        lines[lineSlot].centerImage = center;
        lines[lineSlot].directionImage = {
            stableDirection.x() / directionLength,
            stableDirection.y() / directionLength,
        };
        lines[lineSlot].handleImage = visibleHandlePoint(center, lines[lineSlot].directionImage);
        lines[lineSlot].directionPatientUnit = directionPatient;
        ++lineSlot;
        if (lineSlot >= static_cast<int>(lines.size())) {
            break;
        }
    }
    return lines;
}

QPointF MprSliceWidget::visibleHandlePoint(QPointF centerImage, QPointF directionImage) const
{
    const double dirLength = std::hypot(directionImage.x(), directionImage.y());
    if (dirLength <= 1.0e-6 || m_image.isNull()) {
        return centerImage;
    }

    const QPointF directionUnit{
        directionImage.x() / dirLength,
        directionImage.y() / dirLength,
    };

    const double minX = kRotationHandleMarginPx;
    const double maxX = static_cast<double>(std::max(m_image.width() - 1, 0)) - kRotationHandleMarginPx;
    const double minY = kRotationHandleMarginPx;
    const double maxY = static_cast<double>(std::max(m_image.height() - 1, 0)) - kRotationHandleMarginPx;

    double maxDistance = std::numeric_limits<double>::infinity();
    if (directionUnit.x() > 1.0e-6) {
        maxDistance = std::min(maxDistance, (maxX - centerImage.x()) / directionUnit.x());
    } else if (directionUnit.x() < -1.0e-6) {
        maxDistance = std::min(maxDistance, (minX - centerImage.x()) / directionUnit.x());
    }
    if (directionUnit.y() > 1.0e-6) {
        maxDistance = std::min(maxDistance, (maxY - centerImage.y()) / directionUnit.y());
    } else if (directionUnit.y() < -1.0e-6) {
        maxDistance = std::min(maxDistance, (minY - centerImage.y()) / directionUnit.y());
    }

    if (!std::isfinite(maxDistance)) {
        maxDistance = kRotationHandleDistancePx;
    }
    maxDistance = std::max(0.0, maxDistance);
    const double targetDistance = std::min(maxDistance, std::max(kRotationHandleDistancePx, maxDistance * 0.92));
    return clampImagePoint({
        centerImage.x() + directionUnit.x() * targetDistance,
        centerImage.y() + directionUnit.y() * targetDistance,
    });
}

std::pair<QPointF, QPointF> MprSliceWidget::visibleCrosslineEndpoints(const CrosslineInfo& line) const
{
    const double dirLength = std::hypot(line.directionImage.x(), line.directionImage.y());
    if (dirLength <= 1.0e-6 || m_image.isNull()) {
        return {line.centerImage, line.centerImage};
    }

    const QPointF directionUnit{
        line.directionImage.x() / dirLength,
        line.directionImage.y() / dirLength,
    };
    const double minX = kOrientationLabelMarginPx;
    const double maxX = static_cast<double>(std::max(m_image.width() - 1, 0)) - kOrientationLabelMarginPx;
    const double minY = kOrientationLabelMarginPx;
    const double maxY = static_cast<double>(std::max(m_image.height() - 1, 0)) - kOrientationLabelMarginPx;

    const auto maxDistanceAlong = [&](QPointF direction) {
        double maxDistance = std::numeric_limits<double>::infinity();
        if (direction.x() > 1.0e-6) {
            maxDistance = std::min(maxDistance, (maxX - line.centerImage.x()) / direction.x());
        } else if (direction.x() < -1.0e-6) {
            maxDistance = std::min(maxDistance, (minX - line.centerImage.x()) / direction.x());
        }
        if (direction.y() > 1.0e-6) {
            maxDistance = std::min(maxDistance, (maxY - line.centerImage.y()) / direction.y());
        } else if (direction.y() < -1.0e-6) {
            maxDistance = std::min(maxDistance, (minY - line.centerImage.y()) / direction.y());
        }
        return std::max(0.0, std::isfinite(maxDistance) ? maxDistance : 0.0);
    };

    const double positiveDistance = maxDistanceAlong(directionUnit);
    const double negativeDistance = maxDistanceAlong({-directionUnit.x(), -directionUnit.y()});
    return {
        clampImagePoint({
            line.centerImage.x() - directionUnit.x() * negativeDistance,
            line.centerImage.y() - directionUnit.y() * negativeDistance,
        }),
        clampImagePoint({
            line.centerImage.x() + directionUnit.x() * positiveDistance,
            line.centerImage.y() + directionUnit.y() * positiveDistance,
        }),
    };
}

std::pair<QPointF, QPointF> MprSliceWidget::crosslineHandleCenters(const CrosslineInfo& line) const
{
    const auto [start, end] = visibleCrosslineEndpoints(line);
    const double dirLength = std::hypot(line.directionImage.x(), line.directionImage.y());
    if (dirLength <= 1.0e-6) {
        return {start, end};
    }

    const QPointF directionUnit{
        line.directionImage.x() / dirLength,
        line.directionImage.y() / dirLength,
    };
    return {
        clampImagePoint({
            start.x() + directionUnit.x() * kRotationHandleInsetPx,
            start.y() + directionUnit.y() * kRotationHandleInsetPx,
        }),
        clampImagePoint({
            end.x() - directionUnit.x() * kRotationHandleInsetPx,
            end.y() - directionUnit.y() * kRotationHandleInsetPx,
        }),
    };
}

void MprSliceWidget::drawCrossline(
    QPainter& painter,
    const CrosslineInfo& line,
    QColor color,
    bool drawHandle,
    bool drawDirectionCue)
{
    if (std::hypot(line.directionImage.x(), line.directionImage.y()) <= 1.0e-6) {
        return;
    }

    const auto [start, end] = visibleCrosslineEndpoints(line);

    painter.setPen(QPen(color, 1.5));
    painter.drawLine(start, end);
    drawCrosslineOrientationLabels(painter, line, color);
    if (drawHandle) {
        const auto [startHandle, endHandle] = crosslineHandleCenters(line);
        drawRotationHandle(painter, startHandle, line.directionImage, color);
        drawRotationHandle(painter, endHandle, line.directionImage, color);
        if (drawDirectionCue) {
            drawInstrumentDirectionCue(
                painter,
                startHandle,
                {-line.directionImage.x(), -line.directionImage.y()},
                QColor(255, 247, 150, 250));
        }
    }
}

void MprSliceWidget::drawCrosslineOrientationLabels(QPainter& painter, const CrosslineInfo& line, QColor color)
{
    const QString negativeLabel = orientationLabelForPatientVector(line.directionPatientUnit * -1.0);
    const QString positiveLabel = orientationLabelForPatientVector(line.directionPatientUnit);
    if (negativeLabel.isEmpty() && positiveLabel.isEmpty()) {
        return;
    }

    const auto [start, end] = visibleCrosslineEndpoints(line);
    const double dirLength = std::hypot(line.directionImage.x(), line.directionImage.y());
    if (dirLength <= 1.0e-6) {
        return;
    }

    const QPointF directionUnit{
        line.directionImage.x() / dirLength,
        line.directionImage.y() / dirLength,
    };
    const auto drawLabel = [&](QPointF anchor, QPointF direction, const QString& label) {
        if (label.isEmpty()) {
            return;
        }

        const QPointF center = clampImagePoint({
            anchor.x() - direction.x() * kOrientationLabelInsetPx,
            anchor.y() - direction.y() * kOrientationLabelInsetPx,
        });
        painter.save();
        QFont font = painter.font();
        font.setBold(true);
        font.setPointSizeF(std::max(8.0, font.pointSizeF() > 0.0 ? font.pointSizeF() - 0.5 : 9.0));
        painter.setFont(font);
        const QFontMetrics metrics(font);
        QRect textRect = metrics.boundingRect(label);
        textRect.adjust(-6, -3, 6, 3);
        textRect.moveCenter(QPoint(
            static_cast<int>(std::llround(center.x())),
            static_cast<int>(std::llround(center.y()))));
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(10, 12, 16, 190));
        painter.drawRoundedRect(textRect, 4.0, 4.0);
        painter.setPen(QPen(color.lighter(130)));
        painter.drawText(textRect, Qt::AlignCenter, label);
        painter.restore();
    };

    drawLabel(start, {-directionUnit.x(), -directionUnit.y()}, negativeLabel);
    drawLabel(end, directionUnit, positiveLabel);
}

void MprSliceWidget::drawRotationHandle(QPainter& painter, QPointF centerImage, QPointF directionImage, QColor color)
{
    const double dirLength = std::hypot(directionImage.x(), directionImage.y());
    if (dirLength <= 1.0e-6) {
        return;
    }

    const QPointF tangent{
        directionImage.x() / dirLength,
        directionImage.y() / dirLength,
    };
    const QPointF normal{-tangent.y(), tangent.x()};
    const QPointF offset{
        tangent.x() * (kRotationHandleBarGapPx * 0.5),
        tangent.y() * (kRotationHandleBarGapPx * 0.5),
    };
    const QPointF barHalf{
        normal.x() * (kRotationHandleBarLengthPx * 0.5),
        normal.y() * (kRotationHandleBarLengthPx * 0.5),
    };

    painter.save();
    painter.setPen(QPen(color, 2.0));
    painter.drawLine(
        QPointF(centerImage.x() - offset.x() - barHalf.x(), centerImage.y() - offset.y() - barHalf.y()),
        QPointF(centerImage.x() - offset.x() + barHalf.x(), centerImage.y() - offset.y() + barHalf.y()));
    painter.drawLine(
        QPointF(centerImage.x() + offset.x() - barHalf.x(), centerImage.y() + offset.y() - barHalf.y()),
        QPointF(centerImage.x() + offset.x() + barHalf.x(), centerImage.y() + offset.y() + barHalf.y()));
    painter.restore();
}

void MprSliceWidget::drawInstrumentDirectionCue(
    QPainter& painter,
    QPointF handleCenterImage,
    QPointF directionImage,
    QColor color)
{
    const double dirLength = std::hypot(directionImage.x(), directionImage.y());
    if (dirLength <= 1.0e-6) {
        return;
    }

    const QPointF directionUnit{
        directionImage.x() / dirLength,
        directionImage.y() / dirLength,
    };
    const QPointF normal{-directionUnit.y(), directionUnit.x()};
    const QPointF tip{
        handleCenterImage.x() - directionUnit.x() * 8.0,
        handleCenterImage.y() - directionUnit.y() * 8.0,
    };
    const QPointF tailCenter{
        tip.x() - directionUnit.x() * 16.0,
        tip.y() - directionUnit.y() * 16.0,
    };
    const QPointF firstTail{
        tailCenter.x() + normal.x() * 8.0,
        tailCenter.y() + normal.y() * 8.0,
    };
    const QPointF secondTail{
        tailCenter.x() - normal.x() * 8.0,
        tailCenter.y() - normal.y() * 8.0,
    };

    painter.save();
    QPen shadowPen(QColor(8, 10, 14, 210), 5.0);
    shadowPen.setCapStyle(Qt::RoundCap);
    shadowPen.setJoinStyle(Qt::RoundJoin);
    painter.setPen(shadowPen);
    painter.drawLine(firstTail, tip);
    painter.drawLine(secondTail, tip);

    QPen cuePen(color, 2.8);
    cuePen.setCapStyle(Qt::RoundCap);
    cuePen.setJoinStyle(Qt::RoundJoin);
    painter.setPen(cuePen);
    painter.drawLine(firstTail, tip);
    painter.drawLine(secondTail, tip);
    painter.restore();
}

void MprSliceWidget::anchorCrosshairAtImagePoint(QPointF imagePoint)
{
    const auto params = parameters();
    if (!params.ok()) {
        return;
    }

    const measurement::MprSliceRequest& request = params.value().request;
    const QPointF clamped = clampImagePoint(imagePoint);
    const double centerX = static_cast<double>(request.outputWidth) * 0.5;
    const double centerY = static_cast<double>(request.outputHeight) * 0.5;
    m_pan.x = -(clamped.x() - centerX) * request.pixelSpacingMm;
    m_pan.y = (clamped.y() - centerY) * request.pixelSpacingMm;
}

QPointF MprSliceWidget::widgetPointToImagePoint(const QPoint& position) const
{
    const QRect rect = imageRect();
    if (rect.width() <= 0 || rect.height() <= 0 || m_image.isNull()) {
        return {};
    }

    const double normalizedX = clampDouble(
        static_cast<double>(position.x() - rect.left()) / static_cast<double>(rect.width()),
        0.0,
        1.0);
    const double normalizedY = clampDouble(
        static_cast<double>(position.y() - rect.top()) / static_cast<double>(rect.height()),
        0.0,
        1.0);
    return clampImagePoint({
        normalizedX * static_cast<double>(std::max(m_image.width() - 1, 0)),
        normalizedY * static_cast<double>(std::max(m_image.height() - 1, 0)),
    });
}

MprSliceWidget::InteractionTarget MprSliceWidget::hitTestCrosshair(const QPoint& position) const
{
    if (m_image.isNull() || m_state == nullptr || !imageRect().contains(position)) {
        return InteractionTarget::None;
    }

    const QPointF imagePoint = widgetPointToImagePoint(position);
    const QPointF crosshair = patientToImagePoint(m_state->crosshairPatientMm);
    const double dx = imagePoint.x() - crosshair.x();
    const double dy = imagePoint.y() - crosshair.y();
    if (std::hypot(dx, dy) <= kCrosshairCenterRadiusPx) {
        return InteractionTarget::Center;
    }

    const auto lines = crosslines();
    for (size_t index = 0; index < lines.size(); ++index) {
        const auto [startHandle, endHandle] = crosslineHandleCenters(lines[index]);
        if (std::hypot(
                imagePoint.x() - startHandle.x(),
                imagePoint.y() - startHandle.y())
                <= kRotationHandleRadiusPx + 2.0
            || std::hypot(
                imagePoint.x() - endHandle.x(),
                imagePoint.y() - endHandle.y())
                <= kRotationHandleRadiusPx + 2.0) {
            return index == 0 ? InteractionTarget::FirstHandle : InteractionTarget::SecondHandle;
        }
    }
    return InteractionTarget::None;
}

void MprSliceWidget::updateCursorForHover(const QPoint& position)
{
    if (m_dragMode != DragMode::None) {
        return;
    }

    if ((QApplication::keyboardModifiers() & Qt::ControlModifier) != 0) {
        setCursor(Qt::SizeAllCursor);
        return;
    }

    switch (hitTestCrosshair(position)) {
    case InteractionTarget::Center:
        setCursor(Qt::SizeAllCursor);
        break;
    case InteractionTarget::FirstHandle:
    case InteractionTarget::SecondHandle:
        setCursor(Qt::OpenHandCursor);
        break;
    case InteractionTarget::None:
        setCursor(Qt::ArrowCursor);
        break;
    }
}

QPointF MprSliceWidget::patientToImagePoint(measurement::Vec3d patient) const
{
    const auto params = parameters();
    if (!params.ok()) {
        return {};
    }

    const measurement::MprSliceFrame& frame = params.value().frame;
    const measurement::MprSliceRequest& request = params.value().request;
    const measurement::Vec3d delta = patient - frame.originPatientMm;
    const double u = measurement::dot(delta, frame.horizontalPatientUnit);
    const double v = measurement::dot(delta, frame.verticalPatientUnit);
    return {
        static_cast<double>(request.outputWidth) * 0.5 + u / request.pixelSpacingMm,
        static_cast<double>(request.outputHeight) * 0.5 - v / request.pixelSpacingMm,
    };
}

measurement::Vec3d MprSliceWidget::imagePointToPatient(QPointF imagePoint) const
{
    const auto params = parameters();
    if (!params.ok()) {
        return {};
    }

    const measurement::MprSliceFrame& frame = params.value().frame;
    const measurement::MprSliceRequest& request = params.value().request;
    const double u = (imagePoint.x() - static_cast<double>(request.outputWidth) * 0.5) * request.pixelSpacingMm;
    const double v = (static_cast<double>(request.outputHeight) * 0.5 - imagePoint.y()) * request.pixelSpacingMm;
    return frame.originPatientMm + frame.horizontalPatientUnit * u + frame.verticalPatientUnit * v;
}

void MprSliceWidget::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.fillRect(rect(), QColor(18, 20, 24));
    const QRect rectForImage = imageRect();

    painter.setPen(QColor(230, 230, 230));
    painter.drawText(10, 20, planeTitle(m_plane));

    if (m_image.isNull()) {
        painter.setPen(QColor(170, 170, 170));
    painter.drawText(rectForImage, Qt::AlignCenter, m_renderStatus.isEmpty() ? "没有可渲染的体数据" : m_renderStatus);
        return;
    }

    painter.drawImage(rectForImage, m_image);
    painter.save();
    painter.setClipRect(rectForImage);
    painter.translate(rectForImage.topLeft());
    painter.scale(
        static_cast<double>(rectForImage.width()) / static_cast<double>(m_image.width()),
        static_cast<double>(rectForImage.height()) / static_cast<double>(m_image.height()));

    const auto params = parameters();
    if (params.ok()) {
        const QPointF crosshair = patientToImagePoint(m_state->crosshairPatientMm);
        const auto lines = crosslines();
        const bool drawYellowLineDirectionCue =
            m_plane == measurement::MprPlane::Axial || m_plane == measurement::MprPlane::Sagittal;
        drawCrossline(painter, lines[0], QColor(255, 180, 0, 220), true, drawYellowLineDirectionCue);
        drawCrossline(painter, lines[1], QColor(0, 200, 255, 220), true);
        painter.setBrush(QColor(0, 255, 180, 220));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(crosshair, 4.0, 4.0);
        drawInstrumentOverlays(painter);
        drawMeasurementOverlays(painter);
    }
    painter.restore();

    painter.setPen(QColor(80, 90, 100));
    painter.drawRect(rectForImage);
}

void MprSliceWidget::drawInstrumentOverlays(QPainter& painter)
{
    for (const InstrumentRenderSection& section : m_instrumentSections) {
        const InstrumentRenderStyle& style = section.segment.style;
        const QColor color = QColor::fromRgbF(style.color[0], style.color[1], style.color[2], style.opacity);
        QPen pen(color, style.lineWidth);
        painter.setPen(pen);
        painter.setBrush(Qt::NoBrush);
        for (const measurement::InstrumentSectionSegment& segment : section.segments) {
            painter.drawLine(
                patientToImagePoint(segment.startPatientMm),
                patientToImagePoint(segment.endPatientMm));
        }
    }
}

void MprSliceWidget::mousePressEvent(QMouseEvent* event)
{
    if (m_activated) {
        m_activated(m_plane);
    }
    if (m_image.isNull() || m_state == nullptr || m_volume == nullptr) {
        event->ignore();
        return;
    }

    m_lastMousePosition = event->pos();
    if (event->button() == Qt::LeftButton && (event->modifiers() & Qt::ControlModifier) != 0) {
        beginWindowLevelDrag(event->pos());
        event->accept();
        return;
    }

    if (event->button() == Qt::LeftButton) {
        if (m_measurementMode != measurement::MeasurementMode::Navigate && m_measurementPointAdded) {
            const auto patientPoint = patientPointFromWidgetPosition(event->pos());
            const auto slicePlane = currentMeasurementPlane();
            if (patientPoint.has_value() && slicePlane.has_value()) {
                m_measurementPointAdded(m_plane, *patientPoint, *slicePlane);
                event->accept();
                return;
            }
        }

        const InteractionTarget target = hitTestCrosshair(event->pos());
        if (target == InteractionTarget::Center) {
            beginCrosshairDrag(target, event->pos());
            event->accept();
            return;
        }
        if (target == InteractionTarget::FirstHandle || target == InteractionTarget::SecondHandle) {
            beginRotationDrag(target, event->pos());
            event->accept();
            return;
        }
        if (imageRect().contains(event->pos())) {
            beginCrosshairDrag(InteractionTarget::Center, event->pos());
            updateCrosshairDrag(event->pos());
            event->accept();
            return;
        }
        event->ignore();
        return;
    }

    if (event->button() == Qt::MiddleButton) {
        beginPanDrag(event->pos());
        event->accept();
        return;
    }

    if (event->button() == Qt::RightButton) {
        if (m_measurementMode != measurement::MeasurementMode::Navigate
            && !m_pendingMeasurementPoints.empty()
            && m_measurementCancelRequested) {
            m_measurementCancelRequested();
            event->accept();
            return;
        }
        beginZoomDrag(event->pos());
        event->accept();
        return;
    }

    event->ignore();
}

void MprSliceWidget::mouseMoveEvent(QMouseEvent* event)
{
    switch (m_dragMode) {
    case DragMode::CrosshairCenter:
        updateCrosshairDrag(event->pos());
        event->accept();
        return;
    case DragMode::RotateFirstLine:
    case DragMode::RotateSecondLine:
        updateRotationDrag(event->pos());
        event->accept();
        return;
    case DragMode::Pan:
        updatePanDrag(event->pos());
        event->accept();
        return;
    case DragMode::Zoom:
        updateZoomDrag(event->pos());
        event->accept();
        return;
    case DragMode::WindowLevel:
        updateWindowLevelDrag(event->pos());
        event->accept();
        return;
    case DragMode::None:
        break;
    }

    m_lastMousePosition = event->pos();
    if (m_measurementMode != measurement::MeasurementMode::Navigate && m_measurementHoverChanged) {
        m_measurementHoverChanged(m_plane, patientPointFromWidgetPosition(event->pos()));
    }
    updateCursorForHover(event->pos());
    event->ignore();
}

void MprSliceWidget::mouseReleaseEvent(QMouseEvent* event)
{
    if (m_dragMode != DragMode::None) {
        m_dragMode = DragMode::None;
        updateCursorForHover(event->pos());
        event->accept();
        return;
    }
    event->ignore();
}

void MprSliceWidget::wheelEvent(QWheelEvent* event)
{
    if (m_activated) {
        m_activated(m_plane);
    }
    if (m_image.isNull() || m_volume == nullptr || m_state == nullptr || !imageRect().contains(event->position().toPoint())) {
        event->ignore();
        return;
    }

    const int steps = event->angleDelta().y() / 120;
    if (steps == 0) {
        event->ignore();
        return;
    }
    stepSlice(steps);
    event->accept();
}

void MprSliceWidget::beginCrosshairDrag(InteractionTarget target, const QPoint& position)
{
    m_dragStartPosition = position;
    m_dragStartCrosshairPatientMm = m_state != nullptr ? m_state->crosshairPatientMm : measurement::Vec3d{};
    switch (target) {
    case InteractionTarget::Center:
        m_dragMode = DragMode::CrosshairCenter;
        setCursor(Qt::ClosedHandCursor);
        break;
    case InteractionTarget::FirstHandle:
    case InteractionTarget::SecondHandle:
    case InteractionTarget::None:
        m_dragMode = DragMode::None;
        break;
    }
}

void MprSliceWidget::updateCrosshairDrag(const QPoint& position)
{
    if (m_state == nullptr || m_crosshairChanged == nullptr) {
        return;
    }

    const auto params = parameters();
    if (!params.ok()) {
        return;
    }

    QPointF desiredImagePoint = widgetPointToImagePoint(position);
    switch (m_dragMode) {
    case DragMode::CrosshairCenter:
        break;
    default:
        return;
    }

    measurement::Vec3d patientPoint = imagePointToPatient(desiredImagePoint);
    if (!isFiniteVec(patientPoint)) {
        return;
    }

    anchorCrosshairAtImagePoint(desiredImagePoint);
    m_crosshairChanged(patientPoint);
}

void MprSliceWidget::beginRotationDrag(InteractionTarget target, const QPoint& position)
{
    if (m_state == nullptr) {
        return;
    }

    const QPointF center = patientToImagePoint(m_state->crosshairPatientMm);
    const QPointF imagePoint = widgetPointToImagePoint(position);
    m_lastRotationAngleRad = std::atan2(
        imagePoint.y() - center.y(),
        imagePoint.x() - center.x());
    m_dragMode = target == InteractionTarget::FirstHandle
        ? DragMode::RotateFirstLine
        : DragMode::RotateSecondLine;
    setCursor(Qt::ClosedHandCursor);
}

void MprSliceWidget::updateRotationDrag(const QPoint& position)
{
    if (m_state == nullptr || m_planeRotationChanged == nullptr) {
        return;
    }

    const QPointF center = patientToImagePoint(m_state->crosshairPatientMm);
    const QPointF imagePoint = widgetPointToImagePoint(position);
    const double angle = std::atan2(
        imagePoint.y() - center.y(),
        imagePoint.x() - center.x());
    double delta = angle - m_lastRotationAngleRad;
    while (delta > kPi) {
        delta -= 2.0 * kPi;
    }
    while (delta < -kPi) {
        delta += 2.0 * kPi;
    }
    m_lastRotationAngleRad = angle;
    if (std::abs(delta) <= 1.0e-6) {
        return;
    }
    const auto lines = crosslines();
    const measurement::MprPlane linePlane = m_dragMode == DragMode::RotateFirstLine
        ? lines[0].sourcePlane
        : lines[1].sourcePlane;
    m_planeRotationChanged(m_plane, linePlane, -delta);
}

void MprSliceWidget::beginPanDrag(const QPoint& position)
{
    m_dragMode = DragMode::Pan;
    m_dragStartPosition = position;
    m_dragStartPan = m_pan;
    setCursor(Qt::ClosedHandCursor);
}

void MprSliceWidget::updatePanDrag(const QPoint& position)
{
    const auto params = parameters();
    const QRect rectForImage = imageRect();
    if (!params.ok() || rectForImage.width() <= 0 || rectForImage.height() <= 0) {
        return;
    }

    const QPoint delta = position - m_dragStartPosition;
    const double imageDx = static_cast<double>(delta.x()) / static_cast<double>(rectForImage.width()) * static_cast<double>(m_image.width());
    const double imageDy = static_cast<double>(delta.y()) / static_cast<double>(rectForImage.height()) * static_cast<double>(m_image.height());
    m_pan.x = m_dragStartPan.x - imageDx * params.value().request.pixelSpacingMm;
    m_pan.y = m_dragStartPan.y + imageDy * params.value().request.pixelSpacingMm;
    refreshImage();
}

void MprSliceWidget::beginZoomDrag(const QPoint& position)
{
    m_dragMode = DragMode::Zoom;
    m_dragStartPosition = position;
    m_dragStartZoom = m_zoom;
    setCursor(Qt::SizeVerCursor);
}

void MprSliceWidget::updateZoomDrag(const QPoint& position)
{
    const int dy = position.y() - m_dragStartPosition.y();
    m_zoom = clampZoom(m_dragStartZoom * std::exp(-static_cast<double>(dy) * kZoomDragSensitivity));
    refreshImage();
}

void MprSliceWidget::beginWindowLevelDrag(const QPoint& position)
{
    m_dragMode = DragMode::WindowLevel;
    m_dragStartPosition = position;
    m_dragStartWindowCenterHu = m_windowCenterHu;
    m_dragStartWindowWidthHu = m_windowWidthHu;
    setCursor(Qt::SizeAllCursor);
}

void MprSliceWidget::updateWindowLevelDrag(const QPoint& position)
{
    const QPoint delta = position - m_dragStartPosition;
    m_windowCenterHu = m_dragStartWindowCenterHu - static_cast<double>(delta.y()) * kWindowCenterSensitivityHuPerPixel;
    m_windowWidthHu = std::max(1.0, m_dragStartWindowWidthHu + static_cast<double>(delta.x()) * kWindowWidthSensitivityHuPerPixel);
    if (m_windowLevelChanged) {
        m_windowLevelChanged(m_windowCenterHu, m_windowWidthHu);
    } else {
        refreshImage();
    }
}

void MprSliceWidget::stepSlice(int steps)
{
    if (m_state == nullptr || m_volume == nullptr || m_crosshairChanged == nullptr || steps == 0) {
        return;
    }

    const auto params = parameters();
    if (!params.ok()) {
        return;
    }

    const measurement::MprSliceFrame& frame = params.value().frame;
    const measurement::VolumeMetadata& metadata = m_volume->metadata;
    const measurement::Vec3d row = measurement::normalize(metadata.rowDirectionPatient);
    const measurement::Vec3d column = measurement::normalize(metadata.columnDirectionPatient);
    const measurement::Vec3d slice = measurement::normalize(metadata.sliceDirectionPatient);
    const measurement::Vec3d normal = measurement::normalize(frame.normalPatientUnit);
    const double voxelRate = std::hypot(
        measurement::dot(normal, row) / metadata.spacingMm.x,
        measurement::dot(normal, column) / metadata.spacingMm.y,
        measurement::dot(normal, slice) / metadata.spacingMm.z);
    if (!std::isfinite(voxelRate) || voxelRate <= 1.0e-9) {
        return;
    }

    const double stepMm = static_cast<double>(steps) / voxelRate;
    const measurement::Vec3d candidatePatient = m_state->crosshairPatientMm + normal * stepMm;
    measurement::Vec3d voxel = measurement::patientToVoxel(m_volume->transform, candidatePatient);
    if (!isFiniteVec(voxel)) {
        return;
    }
    const measurement::Size3i dims = m_volume->metadata.dimensions;
    voxel.x = clampDouble(voxel.x, 0.0, static_cast<double>(dims.x - 1));
    voxel.y = clampDouble(voxel.y, 0.0, static_cast<double>(dims.y - 1));
    voxel.z = clampDouble(voxel.z, 0.0, static_cast<double>(dims.z - 1));
    const measurement::Vec3d patient = measurement::voxelToPatient(m_volume->transform, voxel);
    if (!isFiniteVec(patient)) {
        return;
    }
    m_crosshairChanged(patient);
}
}  // namespace measurement_app
