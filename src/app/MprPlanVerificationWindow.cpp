#include "MprPlanVerificationWindow.h"

#include "DrrInteractionGeometry.h"
#include "InstrumentRenderModel.h"
#include "measurement/dicom/DicomVolumeLoader.h"
#include "measurement/drr/CpuDrrEngine.h"
#include "measurement/drr/CudaDrrEngine.h"
#include "measurement/core/MeasurementVisibility.h"
#include "measurement/persistence/ProjectManifest.h"

#include <QApplication>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QProgressDialog>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <QVTKOpenGLNativeWidget.h>

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkColorTransferFunction.h>
#include <vtkCubeSource.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkImageData.h>
#include <vtkImageProperty.h>
#include <vtkImageSlice.h>
#include <vtkImageSliceMapper.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLineSource.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkPlaneSource.h>
#include <vtkPoints.h>
#include <vtkPiecewiseFunction.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkSmartVolumeMapper.h>
#include <vtkSphereSource.h>
#include <vtkTexture.h>
#include <vtkVolume.h>
#include <vtkVolumeProperty.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <utility>

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
constexpr double kDrrViewportPaddingScale = 1.02;
constexpr int kMaxDrrDetectorSamples = 4096;
constexpr double kPi = 3.14159265358979323846;

struct MeasurementAngleArcInfo {
    bool valid = false;
    QPointF center;
    double radius = 0.0;
    double start = 0.0;
    double span = 0.0;
    QPointF endpointA;
    QPointF endpointB;
};

[[nodiscard]] QString summarizeLoadFailure(const QString& folder, const measurement::ErrorInfo& error)
{
    const QString detail = error.detail.empty()
        ? QString::fromStdString(error.message)
        : QString::fromStdString(error.detail);
    return QString("%1 -> %2: %3")
        .arg(folder, QString::fromStdString(error.code), detail);
}

[[nodiscard]] const char* planeTitle(measurement::MprPlane plane)
{
    switch (plane) {
    case measurement::MprPlane::Axial:
        return "Axial";
    case measurement::MprPlane::Sagittal:
        return "Sagittal";
    case measurement::MprPlane::Coronal:
        return "Coronal";
    }
    return "MPR";
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

[[nodiscard]] const char* measurementModeName(measurement::MeasurementMode mode)
{
    switch (mode) {
    case measurement::MeasurementMode::Navigate:
        return "Navigate";
    case measurement::MeasurementMode::Distance:
        return "Distance";
    case measurement::MeasurementMode::Angle:
        return "Angle";
    }
    return "Navigate";
}

[[nodiscard]] const char* measurementTypeName(measurement::MeasurementType type)
{
    return type == measurement::MeasurementType::Angle ? "Angle" : "Distance";
}

[[nodiscard]] double clampDouble(double value, double minValue, double maxValue)
{
    return std::max(minValue, std::min(maxValue, value));
}

[[nodiscard]] double normalizeAngleRad(double angle)
{
    const double twoPi = kPi * 2.0;
    while (angle < 0.0) {
        angle += twoPi;
    }
    while (angle >= twoPi) {
        angle -= twoPi;
    }
    return angle;
}

[[nodiscard]] double shortestAngleDeltaRad(double start, double end)
{
    double delta = normalizeAngleRad(end) - normalizeAngleRad(start);
    if (delta > kPi) {
        delta -= kPi * 2.0;
    } else if (delta < -kPi) {
        delta += kPi * 2.0;
    }
    return delta;
}

[[nodiscard]] double arcMidAngleRad(double start, double span)
{
    return start + span * 0.5;
}

[[nodiscard]] double distanceToPoint(QPointF first, QPointF second)
{
    return std::hypot(first.x() - second.x(), first.y() - second.y());
}

[[nodiscard]] MeasurementAngleArcInfo angleArcInfo(const std::vector<QPointF>& points, QSizeF bounds)
{
    MeasurementAngleArcInfo arc;
    if (points.size() != 4) {
        return arc;
    }

    const QPointF p1 = points[0];
    const QPointF p2 = points[1];
    const QPointF p3 = points[2];
    const QPointF p4 = points[3];
    const double denominator =
        (p1.x() - p2.x()) * (p3.y() - p4.y())
        - (p1.y() - p2.y()) * (p3.x() - p4.x());
    if (std::abs(denominator) < 1.0e-6) {
        return arc;
    }

    const double line1 = p1.x() * p2.y() - p1.y() * p2.x();
    const double line2 = p3.x() * p4.y() - p3.y() * p4.x();
    const QPointF center{
        (line1 * (p3.x() - p4.x()) - (p1.x() - p2.x()) * line2) / denominator,
        (line1 * (p3.y() - p4.y()) - (p1.y() - p2.y()) * line2) / denominator,
    };
    if (!std::isfinite(center.x()) || !std::isfinite(center.y())) {
        return arc;
    }
    if (center.x() < -bounds.width()
        || center.x() > bounds.width() * 2.0
        || center.y() < -bounds.height()
        || center.y() > bounds.height() * 2.0) {
        return arc;
    }

    const std::array<QPointF, 2> firstEndpoints{p1, p2};
    const std::array<QPointF, 2> secondEndpoints{p3, p4};
    double bestStart = std::atan2(p1.y() - center.y(), p1.x() - center.x());
    double bestDelta = kPi * 2.0;
    QPointF endpointA = p1;
    QPointF endpointB = p3;
    for (QPointF first : firstEndpoints) {
        const double angleA = std::atan2(first.y() - center.y(), first.x() - center.x());
        for (QPointF second : secondEndpoints) {
            const double angleB = std::atan2(second.y() - center.y(), second.x() - center.x());
            const double delta = shortestAngleDeltaRad(angleA, angleB);
            if (std::abs(delta) < std::abs(bestDelta)) {
                bestDelta = delta;
                bestStart = angleA;
                endpointA = first;
                endpointB = second;
            }
        }
    }

    const double minDistance = std::min({
        distanceToPoint(center, p1),
        distanceToPoint(center, p2),
        distanceToPoint(center, p3),
        distanceToPoint(center, p4),
    });
    const double radius = std::max(16.0, std::min(48.0, std::min(32.0, minDistance / 3.0)));
    if (!std::isfinite(radius) || radius < 1.0) {
        return arc;
    }

    arc.valid = true;
    arc.center = center;
    arc.radius = radius;
    arc.start = bestStart;
    arc.span = bestDelta;
    arc.endpointA = endpointA;
    arc.endpointB = endpointB;
    return arc;
}

void drawMeasurementPolyline(QPainter& painter, const std::vector<QPointF>& points, QColor color, double lineWidth)
{
    if (points.size() < 2) {
        return;
    }

    QPen pen(color, lineWidth);
    pen.setCosmetic(true);
    painter.save();
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);
    for (size_t index = 1; index < points.size(); ++index) {
        painter.drawLine(points[index - 1], points[index]);
    }
    painter.restore();
}

void drawMeasurementHandles(QPainter& painter, const std::vector<QPointF>& points, QColor color)
{
    painter.save();
    QPen pen(color, 2.0);
    pen.setCosmetic(true);
    painter.setPen(pen);
    painter.setBrush(QColor(16, 19, 26));
    for (QPointF point : points) {
        painter.drawEllipse(point, 4.0, 4.0);
    }
    painter.restore();
}

void drawMeasurementLabel(
    QPainter& painter,
    QSizeF bounds,
    const QString& label,
    QPointF anchor,
    QColor color,
    double verticalOffset = 0.0)
{
    if (label.isEmpty()) {
        return;
    }

    painter.save();
    QFont font = painter.font();
    font.setPointSizeF(12.0);
    painter.setFont(font);
    const QFontMetricsF metrics(font);
    constexpr double paddingX = 8.0;
    constexpr double rectHeight = 22.0;
    const double rectWidth = metrics.horizontalAdvance(label) + paddingX * 2.0;
    const double maxX = std::max(4.0, bounds.width() - rectWidth - 4.0);
    const double maxY = std::max(4.0, bounds.height() - rectHeight);
    const double x = clampDouble(anchor.x() + 10.0, 4.0, maxX);
    const double y = clampDouble(anchor.y() - 26.0 + verticalOffset, 4.0, maxY);
    const QRectF rect(x, y, rectWidth, rectHeight);

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(16, 19, 26, 208));
    painter.drawRect(rect);
    QPen outline(color, 1.0);
    outline.setCosmetic(true);
    painter.setPen(outline);
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(rect);
    painter.setPen(Qt::white);
    painter.drawText(rect.adjusted(paddingX, 0.0, -paddingX, 0.0), Qt::AlignVCenter | Qt::AlignLeft, label);
    painter.restore();
}

void drawAngleArc(QPainter& painter, const MeasurementAngleArcInfo& arc, QColor color)
{
    if (!arc.valid) {
        return;
    }

    painter.save();
    QPen extensionPen(color, 1.5);
    extensionPen.setCosmetic(true);
    extensionPen.setDashPattern({5.0, 4.0});
    extensionPen.setColor(QColor(color.red(), color.green(), color.blue(), 184));
    painter.setPen(extensionPen);
    painter.drawLine(arc.endpointA, arc.center);
    painter.drawLine(arc.endpointB, arc.center);

    QPen arcPen(color, 2.0);
    arcPen.setCosmetic(true);
    painter.setPen(arcPen);
    QPainterPath path;
    constexpr int kArcSegments = 36;
    for (int index = 0; index <= kArcSegments; ++index) {
        const double t = static_cast<double>(index) / static_cast<double>(kArcSegments);
        const double angle = arc.start + arc.span * t;
        const QPointF point{
            arc.center.x() + std::cos(angle) * arc.radius,
            arc.center.y() + std::sin(angle) * arc.radius,
        };
        if (index == 0) {
            path.moveTo(point);
        } else {
            path.lineTo(point);
        }
    }
    painter.drawPath(path);
    painter.restore();
}

void drawAngleMeasurement(QPainter& painter, const std::vector<QPointF>& points, QColor color, QSizeF bounds)
{
    if (points.size() != 4) {
        return;
    }

    painter.save();
    QPen pen(color, 2.5);
    pen.setCosmetic(true);
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);
    painter.drawLine(points[0], points[1]);
    painter.drawLine(points[2], points[3]);
    painter.restore();

    drawAngleArc(painter, angleArcInfo(points, bounds), color);
    drawMeasurementHandles(painter, points, color);
}

[[nodiscard]] QPointF angleLabelAnchor(const std::vector<QPointF>& points, QSizeF bounds)
{
    const MeasurementAngleArcInfo arc = angleArcInfo(points, bounds);
    if (arc.valid) {
        const double midAngle = arcMidAngleRad(arc.start, arc.span);
        return {
            arc.center.x() + std::cos(midAngle) * (arc.radius + 12.0),
            arc.center.y() + std::sin(midAngle) * (arc.radius + 12.0),
        };
    }

    if (points.size() >= 4) {
        const QPointF firstMidpoint = (points[0] + points[1]) * 0.5;
        const QPointF secondMidpoint = (points[2] + points[3]) * 0.5;
        return (firstMidpoint + secondMidpoint) * 0.5;
    }

    return points.empty() ? QPointF{} : points.back();
}

void drawMeasurementLabels(
    QPainter& painter,
    QSizeF bounds,
    const std::string& label,
    const std::string& measurementText,
    const std::string& displayText,
    QPointF anchor,
    QColor color)
{
    const QString nameText = QString::fromStdString(label);
    QString valueText = QString::fromStdString(measurementText);
    if (valueText.isEmpty() && nameText.isEmpty()) {
        valueText = QString::fromStdString(displayText);
    }

    const bool hasName = !nameText.isEmpty();
    if (hasName) {
        drawMeasurementLabel(painter, bounds, nameText, anchor, color, -24.0);
    }
    drawMeasurementLabel(painter, bounds, valueText, anchor, color, hasName ? 2.0 : 0.0);
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

[[nodiscard]] int xrayPresetIndex(measurement::XrayPreset preset)
{
    return preset == measurement::XrayPreset::LAT ? 1 : 0;
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

[[nodiscard]] measurement::Vec3d rotateAroundAxis(
    measurement::Vec3d vector,
    measurement::Vec3d axis,
    double angleRad)
{
    const measurement::Vec3d unitAxis = measurement::normalize(axis);
    if (!isFiniteVec(unitAxis) || measurement::length(unitAxis) <= 0.0) {
        return vector;
    }

    const double cosTheta = std::cos(angleRad);
    const double sinTheta = std::sin(angleRad);
    return vector * cosTheta
        + measurement::cross(unitAxis, vector) * sinTheta
        + unitAxis * measurement::dot(unitAxis, vector) * (1.0 - cosTheta);
}

[[nodiscard]] measurement::Vec3d rotatePointAroundAxis(
    measurement::Vec3d point,
    measurement::Vec3d center,
    measurement::Vec3d axis,
    double angleRad)
{
    return center + rotateAroundAxis(point - center, axis, angleRad);
}

[[nodiscard]] measurement::Vec3d reflectAcrossNormal(measurement::Vec3d vector, measurement::Vec3d normal)
{
    const measurement::Vec3d unitNormal = measurement::normalize(normal);
    if (!isFiniteVec(unitNormal) || measurement::length(unitNormal) <= 1.0e-6) {
        return vector;
    }
    return vector - unitNormal * (2.0 * measurement::dot(vector, unitNormal));
}

[[nodiscard]] measurement::Vec3d volumeCenterPatient(const measurement::VolumeData& volume)
{
    const measurement::Size3i dims = volume.metadata.dimensions;
    return measurement::voxelToPatient(
        volume.transform,
        {
            static_cast<double>(dims.x - 1) * 0.5,
            static_cast<double>(dims.y - 1) * 0.5,
            static_cast<double>(dims.z - 1) * 0.5,
        });
}

[[nodiscard]] measurement::MprSliceFrame normalizedFrame(measurement::MprSliceFrame frame)
{
    frame.horizontalPatientUnit = measurement::normalize(frame.horizontalPatientUnit);
    frame.normalPatientUnit = measurement::normalize(frame.normalPatientUnit);
    frame.verticalPatientUnit = measurement::normalize(
        measurement::cross(frame.normalPatientUnit, frame.horizontalPatientUnit));
    frame.horizontalPatientUnit = measurement::normalize(
        measurement::cross(frame.verticalPatientUnit, frame.normalPatientUnit));
    return frame;
}

[[nodiscard]] measurement::Vec3d projectedOntoPlane(measurement::Vec3d vector, measurement::Vec3d normal)
{
    return vector - normal * measurement::dot(vector, normal);
}

[[nodiscard]] measurement::Vec3d fallbackInPlaneAxis(measurement::Vec3d normal)
{
    const std::array<measurement::Vec3d, 3> candidates{
        measurement::Vec3d{1.0, 0.0, 0.0},
        measurement::Vec3d{0.0, 1.0, 0.0},
        measurement::Vec3d{0.0, 0.0, 1.0},
    };
    measurement::Vec3d best{};
    double bestLength = -1.0;
    for (measurement::Vec3d candidate : candidates) {
        const measurement::Vec3d projected = projectedOntoPlane(candidate, normal);
        const double projectedLength = measurement::length(projected);
        if (projectedLength > bestLength) {
            best = projected;
            bestLength = projectedLength;
        }
    }
    return measurement::normalize(best);
}

[[nodiscard]] measurement::MprSliceFrame frameWithNormal(
    measurement::MprSliceFrame frame,
    measurement::Vec3d normal)
{
    const measurement::Vec3d unitNormal = measurement::normalize(normal);
    if (!isFiniteVec(unitNormal) || measurement::length(unitNormal) <= 1.0e-6) {
        return normalizedFrame(frame);
    }

    measurement::Vec3d horizontal = projectedOntoPlane(frame.horizontalPatientUnit, unitNormal);
    if (measurement::length(horizontal) <= 1.0e-6) {
        horizontal = projectedOntoPlane(frame.verticalPatientUnit, unitNormal);
    }
    if (measurement::length(horizontal) <= 1.0e-6) {
        horizontal = fallbackInPlaneAxis(unitNormal);
    }

    frame.normalPatientUnit = unitNormal;
    frame.horizontalPatientUnit = measurement::normalize(horizontal);
    frame.verticalPatientUnit = measurement::normalize(measurement::cross(unitNormal, frame.horizontalPatientUnit));
    frame.horizontalPatientUnit = measurement::normalize(measurement::cross(frame.verticalPatientUnit, unitNormal));
    return frame;
}

[[nodiscard]] measurement::MprPlane remainingPlane(
    measurement::MprPlane first,
    measurement::MprPlane second)
{
    const std::array<measurement::MprPlane, 3> planes{
        measurement::MprPlane::Axial,
        measurement::MprPlane::Sagittal,
        measurement::MprPlane::Coronal,
    };
    for (measurement::MprPlane plane : planes) {
        if (plane != first && plane != second) {
            return plane;
        }
    }
    return measurement::MprPlane::Axial;
}

[[nodiscard]] int clampIndex(double value, int upperExclusive)
{
    return std::clamp(static_cast<int>(std::llround(value)), 0, upperExclusive - 1);
}

[[nodiscard]] QString vecText(measurement::Vec3d value)
{
    return QString("(%1, %2, %3)")
        .arg(value.x, 0, 'f', 2)
        .arg(value.y, 0, 'f', 2)
        .arg(value.z, 0, 'f', 2);
}

class ScopedModalBusyDialog {
public:
    ScopedModalBusyDialog(QWidget* parent, QString labelText, QString windowTitle = "Please Wait")
        : m_dialog(new QProgressDialog(std::move(labelText), QString(), 0, 0, parent))
    {
        m_dialog->setWindowTitle(std::move(windowTitle));
        m_dialog->setWindowModality(Qt::ApplicationModal);
        m_dialog->setCancelButton(nullptr);
        m_dialog->setMinimumDuration(0);
        m_dialog->setAutoClose(false);
        m_dialog->setAutoReset(false);
        m_dialog->setValue(0);
        m_dialog->show();
        QApplication::setOverrideCursor(Qt::WaitCursor);
        QApplication::processEvents();
    }

    ~ScopedModalBusyDialog()
    {
        QApplication::restoreOverrideCursor();
        if (m_dialog != nullptr) {
            m_dialog->close();
        }
        QApplication::processEvents();
    }

    ScopedModalBusyDialog(const ScopedModalBusyDialog&) = delete;
    ScopedModalBusyDialog& operator=(const ScopedModalBusyDialog&) = delete;

private:
    QProgressDialog* m_dialog = nullptr;
};

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

[[nodiscard]] bool buildDrrRenderRequest(
    const measurement::VolumeData* volume,
    measurement::XrayPreset preset,
    const DrrUiSettings& uiSettings,
    measurement::ProjectionParams& projection,
    measurement::DrrRenderSettings& renderSettings)
{
    if (volume == nullptr || !volume->image) {
        return false;
    }

    const measurement::Vec3d boundsMin = volume->transform.boundsMinPatientMm;
    const measurement::Vec3d boundsMax = volume->transform.boundsMaxPatientMm;
    if (!isFiniteVec(boundsMin) || !isFiniteVec(boundsMax)) {
        return false;
    }

    const measurement::Vec3d center = (boundsMin + boundsMax) * 0.5;
    const measurement::Vec3d extent = boundsMax - boundsMin;
    const double maxExtent = std::max({extent.x, extent.y, extent.z, 1.0});
    const double sidMm = std::max(uiSettings.sidMm, 2.0);
    const double sodMm = std::clamp(uiSettings.sodMm, 1.0, sidMm - 1.0e-3);

    projection = {};
    projection.sourcePosPatientMm = center;
    projection.detectorCenterPatientMm = center;
    projection.sidMm = sidMm;
    projection.sodMm = sodMm;

    if (preset == measurement::XrayPreset::LAT) {
        projection.sourcePosPatientMm = center + measurement::Vec3d{-sodMm, 0.0, 0.0};
        projection.detectorCenterPatientMm = center + measurement::Vec3d{sidMm - sodMm, 0.0, 0.0};
        projection.detectorUPatientUnit = {0.0, 1.0, 0.0};
        projection.detectorVPatientUnit = {0.0, 0.0, 1.0};
        projection.primaryAngleDeg = 90.0;
    } else {
        projection.sourcePosPatientMm = center + measurement::Vec3d{0.0, -sodMm, 0.0};
        projection.detectorCenterPatientMm = center + measurement::Vec3d{0.0, sidMm - sodMm, 0.0};
        projection.detectorUPatientUnit = {1.0, 0.0, 0.0};
        projection.detectorVPatientUnit = {0.0, 0.0, 1.0};
        projection.primaryAngleDeg = 0.0;
    }

    const double detectorWidthMm = std::max(uiSettings.detectorWidthMm, 1.0);
    const double detectorHeightMm = std::max(uiSettings.detectorHeightMm, 1.0);
    // The UI detector width/height are the real detector plane dimensions in
    // millimeters.  DRR image resolution is derived from those dimensions and a
    // square detector pixel spacing, so changing width/height changes the
    // physical field of view instead of merely stretching the output image.
    double detectorPixelSpacingMm = uiSettings.pixelSpacingMm > 0.0
        ? uiSettings.pixelSpacingMm
        : kDefaultPixelSpacingMm;
    if (!std::isfinite(detectorPixelSpacingMm) || detectorPixelSpacingMm <= 0.0) {
        detectorPixelSpacingMm = kDefaultPixelSpacingMm;
    }
    // Keep the detector plane dimensions authoritative.  If a very small pixel
    // spacing would exceed the renderer's sample cap, use a coarser effective
    // spacing rather than clipping the physical detector plane.
    detectorPixelSpacingMm = std::max({
        detectorPixelSpacingMm,
        detectorWidthMm / static_cast<double>(kMaxDrrDetectorSamples),
        detectorHeightMm / static_cast<double>(kMaxDrrDetectorSamples),
    });
    renderSettings.width = std::max(1, static_cast<int>(std::llround(detectorWidthMm / detectorPixelSpacingMm)));
    renderSettings.height = std::max(1, static_cast<int>(std::llround(detectorHeightMm / detectorPixelSpacingMm)));
    renderSettings.stepMm = std::max(uiSettings.rayStepMm, 1.0e-6);
    renderSettings.outputLineIntegral = true;
    renderSettings.windowCenter = uiSettings.windowCenter > 0.0 ? uiSettings.windowCenter : maxExtent * 0.55;
    renderSettings.windowWidth = uiSettings.windowWidth > 0.0 ? uiSettings.windowWidth : std::max(maxExtent * 1.1, 1.0);
    renderSettings.gamma = std::max(uiSettings.gamma, 1.0e-6);
    renderSettings.huOffset = uiSettings.huOffset;
    renderSettings.huScale = std::max(uiSettings.huScale, 1.0e-6);

    projection.detectorWidth = renderSettings.width;
    projection.detectorHeight = renderSettings.height;
    projection.pixelSpacingMm = detectorPixelSpacingMm;
    if (!std::isfinite(projection.pixelSpacingMm) || projection.pixelSpacingMm <= 0.0) {
        projection.pixelSpacingMm = 1.0;
    }

    return true;
}

[[nodiscard]] QString instrumentText(const measurement::Instrument& instrument)
{
    const QString type = instrument.type == measurement::InstrumentType::GuidePin ? "Guide pin" : "Pedicle screw";
    const QString flags = QString("%1%2")
                              .arg(instrument.visible ? "visible" : "hidden")
                              .arg(instrument.locked ? ", locked" : "");
    return QString("%1  %2 mm x %3 mm  %4")
        .arg(type)
        .arg(instrument.lengthMm, 0, 'f', 1)
        .arg(instrument.diameterMm, 0, 'f', 1)
        .arg(flags);
}

[[nodiscard]] QDoubleSpinBox* makeSpin(double minValue, double maxValue, double value, double step)
{
    auto* spin = new QDoubleSpinBox();
    spin->setRange(minValue, maxValue);
    spin->setDecimals(3);
    spin->setSingleStep(step);
    spin->setValue(value);
    return spin;
}

[[nodiscard]] measurement::VolumeData makeSyntheticVolume()
{
    measurement::VolumeMetadata metadata;
    metadata.dimensions = {160, 160, 120};
    metadata.spacingMm = {1.2, 1.2, 1.5};
    metadata.originPatientMm = {
        -0.5 * static_cast<double>(metadata.dimensions.x - 1) * metadata.spacingMm.x,
        -0.5 * static_cast<double>(metadata.dimensions.y - 1) * metadata.spacingMm.y,
        -0.5 * static_cast<double>(metadata.dimensions.z - 1) * metadata.spacingMm.z,
    };
    metadata.rowDirectionPatient = {1.0, 0.0, 0.0};
    metadata.columnDirectionPatient = {0.0, 1.0, 0.0};
    metadata.sliceDirectionPatient = {0.0, 0.0, 1.0};
    metadata.rescaleSlope = 1.0;
    metadata.rescaleIntercept = 0.0;

    std::vector<int16_t> voxels;
    voxels.reserve(
        static_cast<size_t>(metadata.dimensions.x)
        * static_cast<size_t>(metadata.dimensions.y)
        * static_cast<size_t>(metadata.dimensions.z));

    int minHu = (std::numeric_limits<int>::max)();
    int maxHu = (std::numeric_limits<int>::lowest)();
    for (int k = 0; k < metadata.dimensions.z; ++k) {
        for (int j = 0; j < metadata.dimensions.y; ++j) {
            for (int i = 0; i < metadata.dimensions.x; ++i) {
                const double x = metadata.originPatientMm.x + static_cast<double>(i) * metadata.spacingMm.x;
                const double y = metadata.originPatientMm.y + static_cast<double>(j) * metadata.spacingMm.y;
                const double z = metadata.originPatientMm.z + static_cast<double>(k) * metadata.spacingMm.z;

                int hu = -1000;
                const double body = (x * x) / (75.0 * 75.0) + (y * y) / (55.0 * 55.0);
                if (body < 1.0) {
                    hu = -120;
                }

                const double vertebralBody = (x * x) / (28.0 * 28.0)
                    + ((y + 8.0) * (y + 8.0)) / (20.0 * 20.0)
                    + (z * z) / (38.0 * 38.0);
                if (vertebralBody < 1.0) {
                    hu = 760;
                }

                const double leftPedicle = ((x + 34.0) * (x + 34.0)) / (9.0 * 9.0)
                    + ((y - 2.0) * (y - 2.0)) / (7.0 * 7.0)
                    + (z * z) / (34.0 * 34.0);
                const double rightPedicle = ((x - 34.0) * (x - 34.0)) / (9.0 * 9.0)
                    + ((y - 2.0) * (y - 2.0)) / (7.0 * 7.0)
                    + (z * z) / (34.0 * 34.0);
                if (leftPedicle < 1.0 || rightPedicle < 1.0) {
                    hu = 1150;
                }

                const double spinous = (x * x) / (7.0 * 7.0)
                    + ((y - 38.0) * (y - 38.0)) / (20.0 * 20.0)
                    + (z * z) / (30.0 * 30.0);
                if (spinous < 1.0) {
                    hu = 900;
                }

                minHu = std::min(minHu, hu);
                maxHu = std::max(maxHu, hu);
                voxels.push_back(static_cast<int16_t>(hu));
            }
        }
    }

    metadata.minHu = minHu;
    metadata.maxHu = maxHu;

    measurement::VolumeData volume;
    volume.metadata = metadata;
    volume.transform = measurement::makeVolumeTransform(metadata).value();
    volume.image = measurement::makeDenseHuVolume(metadata.dimensions, std::move(voxels)).value();
    volume.patientPositionCode = "HFS";
    volume.sourceFolder = "synthetic://pedicle-verification-phantom";
    volume.studyUid = "synthetic-study";
    volume.seriesUid = "synthetic-series";
    volume.dataHash = "synthetic-pedicle-phantom";
    return volume;
}

}  // namespace

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

[[nodiscard]] std::optional<DrrDetectorLine> clipDetectorLineToBounds(
    DrrDetectorPoint point,
    DrrDetectorPoint direction,
    int detectorWidth,
    int detectorHeight)
{
    const double dx = direction.x;
    const double dy = direction.y;
    if (dx * dx + dy * dy <= 1.0e-9 || detectorWidth <= 0 || detectorHeight <= 0) {
        return std::nullopt;
    }

    const double minX = -0.5;
    const double maxX = static_cast<double>(detectorWidth) - 0.5;
    const double minY = -0.5;
    const double maxY = static_cast<double>(detectorHeight) - 0.5;
    std::vector<DrrDetectorPoint> intersections;
    const auto addIfUnique = [&](DrrDetectorPoint candidate) {
        if (candidate.x < minX - 1.0e-6 || candidate.x > maxX + 1.0e-6
            || candidate.y < minY - 1.0e-6 || candidate.y > maxY + 1.0e-6) {
            return;
        }
        for (const DrrDetectorPoint& existing : intersections) {
            if (std::hypot(existing.x - candidate.x, existing.y - candidate.y) <= 1.0e-5) {
                return;
            }
        }
        intersections.push_back(candidate);
    };

    if (std::abs(dx) > 1.0e-9) {
        const double tMinX = (minX - point.x) / dx;
        addIfUnique({minX, point.y + dy * tMinX});
        const double tMaxX = (maxX - point.x) / dx;
        addIfUnique({maxX, point.y + dy * tMaxX});
    }
    if (std::abs(dy) > 1.0e-9) {
        const double tMinY = (minY - point.y) / dy;
        addIfUnique({point.x + dx * tMinY, minY});
        const double tMaxY = (maxY - point.y) / dy;
        addIfUnique({point.x + dx * tMaxY, maxY});
    }

    if (intersections.size() < 2) {
        return std::nullopt;
    }
    return DrrDetectorLine{intersections[0], intersections[1]};
}

[[nodiscard]] DrrDetectorPoint closestDetectorPointOnSegment(
    DrrDetectorPoint point,
    const DrrDetectorLine& segment)
{
    const double dx = segment.tail.x - segment.head.x;
    const double dy = segment.tail.y - segment.head.y;
    const double len2 = dx * dx + dy * dy;
    if (len2 <= 1.0e-9) {
        return point;
    }
    const double t = std::clamp(
        ((point.x - segment.head.x) * dx + (point.y - segment.head.y) * dy) / len2,
        0.0,
        1.0);
    return {segment.head.x + dx * t, segment.head.y + dy * t};
}

[[nodiscard]] std::optional<DrrDetectorLine> projectPatientRayToDetectorConstraint(
    const DrrInteractionRay& sourceRay,
    const measurement::ProjectionParams& targetProjection)
{
    const measurement::Vec3d u = measurement::normalize(targetProjection.detectorUPatientUnit);
    const measurement::Vec3d v = measurement::normalize(targetProjection.detectorVPatientUnit);
    const measurement::Vec3d detectorNormal = measurement::normalize(measurement::cross(u, v));
    const measurement::Vec3d rayDirection = measurement::normalize(sourceRay.directionPatientUnit);
    const measurement::Vec3d sourceToRayOrigin = sourceRay.originPatientMm - targetProjection.sourcePosPatientMm;
    const measurement::Vec3d epipolarNormal = measurement::normalize(measurement::cross(rayDirection, sourceToRayOrigin));
    const measurement::Vec3d lineDirectionPatient = measurement::normalize(measurement::cross(epipolarNormal, detectorNormal));
    if (!isFiniteVec(u) || !isFiniteVec(v) || !isFiniteVec(detectorNormal)
        || !isFiniteVec(epipolarNormal) || !isFiniteVec(lineDirectionPatient)
        || measurement::length(lineDirectionPatient) <= 1.0e-6) {
        return std::nullopt;
    }

    const double epipolarD = measurement::dot(epipolarNormal, sourceRay.originPatientMm);
    const double detectorD = measurement::dot(detectorNormal, targetProjection.detectorCenterPatientMm);
    const measurement::Vec3d pointOnIntersection =
        (measurement::cross(detectorNormal, lineDirectionPatient) * epipolarD
         + measurement::cross(lineDirectionPatient, epipolarNormal) * detectorD)
        / std::max(measurement::dot(lineDirectionPatient, lineDirectionPatient), 1.0e-9);
    if (!isFiniteVec(pointOnIntersection)) {
        return std::nullopt;
    }

    const measurement::Vec3d delta = pointOnIntersection - targetProjection.detectorCenterPatientMm;
    const DrrDetectorPoint detectorPoint{
        measurement::dot(delta, u) / targetProjection.pixelSpacingMm
            + static_cast<double>(targetProjection.detectorWidth) * 0.5
            - 0.5,
        measurement::dot(delta, v) / targetProjection.pixelSpacingMm
            + static_cast<double>(targetProjection.detectorHeight) * 0.5
            - 0.5,
    };
    const DrrDetectorPoint detectorDirection{
        measurement::dot(lineDirectionPatient, u) / targetProjection.pixelSpacingMm,
        measurement::dot(lineDirectionPatient, v) / targetProjection.pixelSpacingMm,
    };
    return clipDetectorLineToBounds(
        detectorPoint,
        detectorDirection,
        targetProjection.detectorWidth,
        targetProjection.detectorHeight);
}

class XrayDisplayWidget final : public QWidget {
public:
    XrayDisplayWidget(QString title, measurement::XrayPreset preset, QWidget* parent = nullptr)
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

    void setPlan(const measurement::SurgicalPlan* plan)
    {
        m_plan = plan;
    }

    void setSelectedInstrumentId(std::string id)
    {
        m_selectedInstrumentId = std::move(id);
    }

    void setVolume(const measurement::VolumeData* volume)
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

    void setDrrSettings(DrrUiSettings settings)
    {
        m_settings = settings;
    }

    void setPlacementActive(bool active)
    {
        m_placementActive = active;
    }

    void setPendingLine(std::optional<DrrDetectorLine> line)
    {
        m_pendingLine = line;
    }

    void setPlacementConstraints(std::array<std::optional<DrrDetectorLine>, 2> constraints)
    {
        m_placementConstraints = constraints;
    }

    void setLineCompletedCallback(std::function<void(measurement::XrayPreset, DrrDetectorLine)> callback)
    {
        m_lineCompleted = std::move(callback);
    }

    void setInstrumentSelectedCallback(std::function<void(std::string)> callback)
    {
        m_instrumentSelected = std::move(callback);
    }

    void setInstrumentDraggedCallback(
        std::function<void(measurement::XrayPreset, std::string, DrrInteractionTarget, DrrDetectorPoint)> callback)
    {
        m_instrumentDragged = std::move(callback);
    }

    void refreshOverlay()
    {
        rebuildVtkScene();
    }

    void refreshDisplaySettings()
    {
        refreshDisplayMappingOnly();
    }

    [[nodiscard]] QImage renderedImage() const
    {
        return m_image;
    }

    void refreshImage()
    {
        m_image = {};
        m_lineIntegral.clear();
        m_lineIntegralWidth = 0;
        m_lineIntegralHeight = 0;
        m_scalarImage = nullptr;
        m_status = "No volume";
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
            m_status = "Invalid X-ray geometry";
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
                m_status = QString("CPU fallback (%1)").arg(QString::fromStdString(cudaVolume.error().code));
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
                               .arg(m_image.isNull() ? "Empty X-ray" : "CUDA DRR")
                               .arg(timer.elapsed());
                rebuildVtkScene();
                return;
            }
            m_status = QString("CPU fallback (%1)").arg(QString::fromStdString(cudaRendered.error().code));
        }

        if (!m_cpuVolumeReady) {
            m_status = QString("CPU DRR volume is not ready | %1 ms").arg(timer.elapsed());
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
        if (m_status.isEmpty() || !m_status.startsWith("CPU fallback")) {
            m_status = m_image.isNull() ? "Empty X-ray" : "CPU DRR";
        }
        m_status = QString("%1 | %2 ms").arg(m_status).arg(timer.elapsed());
        rebuildVtkScene();
    }

protected:
    QSize minimumSizeHint() const override
    {
        return {280, 240};
    }

    bool eventFilter(QObject* watched, QEvent* event) override
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

    [[nodiscard]] bool buildRenderRequest(measurement::ProjectionParams& projection, measurement::DrrRenderSettings& settings) const
    {
        return buildDrrRenderRequest(m_volume, m_preset, m_settings, projection, settings);
    }

    [[nodiscard]] bool currentProjection(measurement::ProjectionParams& projection) const
    {
        measurement::DrrRenderSettings settings;
        return buildRenderRequest(projection, settings);
    }

    bool handleMousePress(QMouseEvent* event)
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

    bool handleMouseMove(QMouseEvent* event)
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

    bool handleMouseRelease(QMouseEvent* event)
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

    [[nodiscard]] QPointF qtToVtkDisplay(QPointF widgetPoint) const
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

    [[nodiscard]] QPointF vtkDisplayToQt(QPointF displayPoint) const
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

    [[nodiscard]] std::optional<measurement::Vec3d> displayToWorld(QPointF displayPoint, double displayZ) const
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

    [[nodiscard]] std::optional<DrrDetectorPoint> widgetToDetector(QPointF widgetPoint) const
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

    [[nodiscard]] std::optional<QPointF> detectorToWidget(DrrDetectorPoint detectorPoint) const
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

    [[nodiscard]] DrrDetectorPoint constrainedDetectorPoint(
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

    [[nodiscard]] std::vector<ProjectedInstrument> projectedInstruments() const
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

    [[nodiscard]] HitResult hitTest(QPointF widgetPoint) const
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

    void updateCursorForHover(QPoint position)
    {
        updateCursorForHover(QPointF(position));
    }

    void updateCursorForHover(QPointF position)
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

    [[nodiscard]] measurement::Vec3d detectorPixelToPatientPoint(
        const measurement::ProjectionParams& projection,
        DrrDetectorPoint point,
        double planeOffsetMm = 0.0) const
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

    [[nodiscard]] vtkSmartPointer<vtkActor> makeDrrHandleActor(
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

    void addDetectorLineActor(
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

    [[nodiscard]] vtkSmartPointer<vtkMatrix4x4> makeImageSliceMatrix(
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

    void configureCamera(const measurement::ProjectionParams& projection)
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
        const double viewAngleDeg = 2.0 * std::atan((neededHeightMm * 0.5 * kDrrViewportPaddingScale) / distanceMm) * 180.0 / kPi;
        camera->SetViewAngle(std::clamp(viewAngleDeg, 1.0, 120.0));
        camera->SetClippingRange(1.0, distanceMm + std::max(2000.0, distanceMm * 2.0));
        m_renderer->ResetCameraClippingRange();
    }

    void rebuildVtkScene()
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

    [[nodiscard]] std::string currentVolumeSignature() const
    {
        if (m_volume == nullptr || !m_volume->image) {
            return {};
        }
        // The app owns one VolumeData object whose address stays stable while loads replace its
        // contents, so the signature must describe both the voxel payload and the current patient
        // geometry. Manual patient-position overrides change the geometry without changing voxels.
        return volumeGeometrySignature(*m_volume, true);
    }

    [[nodiscard]] QImage imageFromLineIntegral(int width, int height, const std::vector<float>& lineIntegral) const
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

    [[nodiscard]] QImage imageFromDrr(const measurement::DrrImage& drr) const
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

    void cacheRenderedDrr(const measurement::DrrImage& drr)
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

    void refreshDisplayMappingOnly()
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

MprPlanVerificationWindow::MprPlanVerificationWindow(QWidget* parent)
    : QMainWindow(parent)
{
    m_planController = std::make_unique<measurement::InstrumentPlanController>(m_plan);
    m_placementController = std::make_unique<measurement::InstrumentPlacementController>(m_plan);
    buildUi();
    loadStartupVolume();
}

void MprPlanVerificationWindow::buildUi()
{
    setWindowTitle("MPR Plan Verification");

    auto* central = new QWidget(this);
    central->setObjectName("AppRoot");
    central->setAttribute(Qt::WA_StyledBackground, true);
    auto* root = new QHBoxLayout(central);
    root->setContentsMargins(12, 12, 12, 8);
    root->setSpacing(0);
    auto* splitter = new QSplitter(Qt::Horizontal, central);
    splitter->setObjectName("MainSplitter");
    root->addWidget(splitter);
    setCentralWidget(central);

    auto* mprPanel = new QWidget(splitter);
    auto* mprLayout = new QGridLayout(mprPanel);
    m_axialView = new MprSliceWidget(measurement::MprPlane::Axial, mprPanel);
    m_sagittalView = new MprSliceWidget(measurement::MprPlane::Sagittal, mprPanel);
    m_coronalView = new MprSliceWidget(measurement::MprPlane::Coronal, mprPanel);
    m_sceneView = new PlanSceneWidget(mprPanel);
    mprLayout->addWidget(m_axialView, 0, 0);
    mprLayout->addWidget(m_sagittalView, 0, 1);
    mprLayout->addWidget(m_coronalView, 1, 0);
    mprLayout->addWidget(m_sceneView, 1, 1);

    auto* xrayPanel = new QWidget(splitter);
    xrayPanel->setObjectName("XrayPanel");
    xrayPanel->setAttribute(Qt::WA_StyledBackground, true);
    auto* xrayLayout = new QVBoxLayout(xrayPanel);
    xrayLayout->setContentsMargins(8, 8, 8, 8);
    xrayLayout->setSpacing(8);
    m_apXrayView = new XrayDisplayWidget("AP X-ray", measurement::XrayPreset::AP, xrayPanel);
    m_latXrayView = new XrayDisplayWidget("LAT X-ray", measurement::XrayPreset::LAT, xrayPanel);
    xrayLayout->addWidget(m_apXrayView, 1);
    xrayLayout->addWidget(m_latXrayView, 1);

    auto* controlPanel = new QWidget(splitter);
    controlPanel->setObjectName("ControlPanel");
    controlPanel->setAttribute(Qt::WA_StyledBackground, true);
    auto* controls = new QVBoxLayout(controlPanel);
    controls->setContentsMargins(8, 8, 8, 8);
    controls->setSpacing(8);

    auto* drrPanel = new QWidget(splitter);
    drrPanel->setObjectName("DrrPanel");
    drrPanel->setAttribute(Qt::WA_StyledBackground, true);
    auto* drrOuterLayout = new QVBoxLayout(drrPanel);
    drrOuterLayout->setContentsMargins(8, 8, 8, 8);
    drrOuterLayout->setSpacing(0);
    auto* drrScrollArea = new QScrollArea(drrPanel);
    drrScrollArea->setWidgetResizable(true);
    drrScrollArea->setFrameShape(QFrame::NoFrame);
    auto* drrContent = new QWidget(drrScrollArea);
    auto* drrPanelLayout = new QVBoxLayout(drrContent);
    drrPanelLayout->setContentsMargins(0, 0, 0, 0);
    drrPanelLayout->setSpacing(8);
    drrScrollArea->setWidget(drrContent);
    drrOuterLayout->addWidget(drrScrollArea);

    auto* workflowGroup = new QGroupBox("Workflow", drrContent);
    auto* loadGrid = new QGridLayout(workflowGroup);
    loadGrid->setHorizontalSpacing(8);
    loadGrid->setVerticalSpacing(8);
    m_loadDicomButton = new QPushButton("Load DICOM", workflowGroup);
    auto* resetViewsButton = new QPushButton("Reset Views", workflowGroup);
    auto* saveButton = new QPushButton("Save .mprproj", workflowGroup);
    m_freeObliqueButton = new QPushButton(m_freeObliqueMode ? "Free oblique: On" : "Free oblique: Off", workflowGroup);
    m_freeObliqueButton->setCheckable(true);
    m_freeObliqueButton->setChecked(m_freeObliqueMode);
    loadGrid->addWidget(m_loadDicomButton, 0, 0, 1, 2);
    loadGrid->addWidget(resetViewsButton, 1, 0);
    loadGrid->addWidget(saveButton, 1, 1);
    loadGrid->addWidget(m_freeObliqueButton, 2, 0, 1, 2);
    drrPanelLayout->addWidget(workflowGroup);

    auto* volumeGroup = new QGroupBox("Volume", drrContent);
    auto* volumeLayout = new QVBoxLayout(volumeGroup);
    m_volumeLabel = new QLabel(volumeGroup);
    m_volumeLabel->setObjectName("VolumeInfoLabel");
    m_volumeLabel->setWordWrap(true);
    volumeLayout->addWidget(m_volumeLabel);
    drrPanelLayout->addWidget(volumeGroup);

    auto* postureGroup = new QGroupBox("Patient position", drrContent);
    auto* postureLayout = new QFormLayout(postureGroup);
    m_patientPostureCombo = new QComboBox(postureGroup);
    m_patientPostureCombo->addItem("Supine", false);
    m_patientPostureCombo->addItem("Prone", true);
    m_headFeetDirectionCombo = new QComboBox(postureGroup);
    m_headFeetDirectionCombo->addItem("Head first", false);
    m_headFeetDirectionCombo->addItem("Feet first", true);
    postureLayout->addRow("Body posture", m_patientPostureCombo);
    postureLayout->addRow("Entry direction", m_headFeetDirectionCombo);
    drrPanelLayout->addWidget(postureGroup);

    auto* crosshairGroup = new QGroupBox("Crosshair voxel", drrContent);
    auto* crosshairLayout = new QGridLayout(crosshairGroup);
    m_xSlider = new QSlider(Qt::Horizontal, crosshairGroup);
    m_ySlider = new QSlider(Qt::Horizontal, crosshairGroup);
    m_zSlider = new QSlider(Qt::Horizontal, crosshairGroup);
    m_xValueLabel = new QLabel(crosshairGroup);
    m_yValueLabel = new QLabel(crosshairGroup);
    m_zValueLabel = new QLabel(crosshairGroup);
    crosshairLayout->addWidget(new QLabel("X", crosshairGroup), 0, 0);
    crosshairLayout->addWidget(m_xSlider, 0, 1);
    crosshairLayout->addWidget(m_xValueLabel, 0, 2);
    crosshairLayout->addWidget(new QLabel("Y", crosshairGroup), 1, 0);
    crosshairLayout->addWidget(m_ySlider, 1, 1);
    crosshairLayout->addWidget(m_yValueLabel, 1, 2);
    crosshairLayout->addWidget(new QLabel("Z", crosshairGroup), 2, 0);
    crosshairLayout->addWidget(m_zSlider, 2, 1);
    crosshairLayout->addWidget(m_zValueLabel, 2, 2);
    drrPanelLayout->addWidget(crosshairGroup);
    // Kept wired for programmatic slice changes. Hidden for now because the
    // MPR views provide direct crosshair manipulation.
    crosshairGroup->setVisible(false);

    auto* planningSplitter = new QSplitter(Qt::Vertical, controlPanel);
    planningSplitter->setChildrenCollapsible(false);
    controls->addWidget(planningSplitter, 1);

    auto* measurementGroup = new QGroupBox("Measurements", controlPanel);
    auto* measurementLayout = new QVBoxLayout(measurementGroup);
    auto* measurementButtons = new QGridLayout();
    measurementButtons->setHorizontalSpacing(8);
    measurementButtons->setVerticalSpacing(8);
    m_measureNavigateButton = new QPushButton("Navigate", measurementGroup);
    m_measureDistanceButton = new QPushButton("Distance", measurementGroup);
    m_measureAngleButton = new QPushButton("Angle", measurementGroup);
    for (QPushButton* button : {m_measureNavigateButton, m_measureDistanceButton, m_measureAngleButton}) {
        button->setCheckable(true);
    }
    measurementButtons->addWidget(m_measureNavigateButton, 0, 0);
    measurementButtons->addWidget(m_measureDistanceButton, 0, 1);
    measurementButtons->addWidget(m_measureAngleButton, 0, 2);
    measurementLayout->addLayout(measurementButtons);
    m_measurementList = new QListWidget(measurementGroup);
    m_measurementList->setMinimumHeight(130);
    measurementLayout->addWidget(m_measurementList, 1);
    auto* measurementForm = new QFormLayout();
    m_measurementLabel = new QLineEdit(measurementGroup);
    measurementForm->addRow("Label", m_measurementLabel);
    measurementLayout->addLayout(measurementForm);
    auto* measurementEditButtons = new QGridLayout();
    auto* deleteMeasurement = new QPushButton("Delete", measurementGroup);
    auto* clearMeasurementsButton = new QPushButton("Clear", measurementGroup);
    measurementEditButtons->addWidget(deleteMeasurement, 0, 0);
    measurementEditButtons->addWidget(clearMeasurementsButton, 0, 1);
    measurementLayout->addLayout(measurementEditButtons);
    planningSplitter->addWidget(measurementGroup);

    auto* instrumentGroup = new QGroupBox("Plan instruments", controlPanel);
    auto* instrumentLayout = new QVBoxLayout(instrumentGroup);
    m_instrumentList = new QListWidget(instrumentGroup);
    m_instrumentList->setMinimumHeight(160);
    instrumentLayout->addWidget(m_instrumentList, 1);

    auto* form = new QFormLayout();
    m_label = new QLineEdit(instrumentGroup);
    m_length = makeSpin(1.0, 300.0, 55.0, 1.0);
    m_diameter = makeSpin(0.5, 20.0, 2.0, 0.5);
    form->addRow("Name", m_label);
    form->addRow("Length mm", m_length);
    form->addRow("Diameter mm", m_diameter);
    instrumentLayout->addLayout(form);

    auto* instrumentButtons = new QGridLayout();
    instrumentButtons->setHorizontalSpacing(8);
    instrumentButtons->setVerticalSpacing(8);
    auto* addPin = new QPushButton("Add pin at crosshair", instrumentGroup);
    auto* addScrew = new QPushButton("Add screw at crosshair", instrumentGroup);
    m_editInstrumentButton = new QPushButton("Edit selected", instrumentGroup);
    auto* remove = new QPushButton("Delete", instrumentGroup);
    instrumentButtons->addWidget(addPin, 0, 0);
    instrumentButtons->addWidget(addScrew, 0, 1);
    instrumentButtons->addWidget(m_editInstrumentButton, 1, 0);
    instrumentButtons->addWidget(remove, 1, 1);
    instrumentLayout->addLayout(instrumentButtons);

    auto* drrPlacementButtons = new QGridLayout();
    drrPlacementButtons->setHorizontalSpacing(8);
    drrPlacementButtons->setVerticalSpacing(8);
    m_drrPinButton = new QPushButton("DRR Pin", instrumentGroup);
    m_drrScrewButton = new QPushButton("DRR Screw", instrumentGroup);
    m_drrCancelButton = new QPushButton("Cancel DRR placement", instrumentGroup);
    m_drrPinButton->setCheckable(true);
    m_drrScrewButton->setCheckable(true);
    drrPlacementButtons->addWidget(m_drrPinButton, 0, 0);
    drrPlacementButtons->addWidget(m_drrScrewButton, 0, 1);
    drrPlacementButtons->addWidget(m_drrCancelButton, 1, 0, 1, 2);
    instrumentLayout->addLayout(drrPlacementButtons);
    planningSplitter->addWidget(instrumentGroup);
    planningSplitter->setSizes({260, 460});
    controlPanel->setMinimumWidth(330);

    auto* statusGroup = new QGroupBox("Status", drrContent);
    auto* statusLayout = new QVBoxLayout(statusGroup);
    m_statusLabel = new QLabel(statusGroup);
    m_statusLabel->setObjectName("StatusInfoLabel");
    m_statusLabel->setWordWrap(true);
    statusLayout->addWidget(m_statusLabel);
    drrPanelLayout->addWidget(statusGroup);
    auto* drrGroup = new QGroupBox("DRR 鍙傛暟", drrPanel);
    auto* drrGroupLayout = new QVBoxLayout(drrGroup);
    drrGroup->setTitle("DRR Parameters");
    drrGroupLayout->setContentsMargins(9, 9, 9, 9);
    drrGroupLayout->setSpacing(8);
    auto* drrTabs = new QTabWidget(drrGroup);

    const auto makeDrrParameterPage = [&](measurement::XrayPreset preset, const QString& title) {
        const int index = xrayPresetIndex(preset);
        auto* page = new QWidget(drrTabs);
        auto* drrForm = new QFormLayout(page);

        m_drrSid[index] = makeSpin(2.0, 1.0e6, 1000.0, 10.0);
        m_drrSid[index]->setDecimals(1);
        m_drrSid[index]->setSuffix(" mm");
        m_drrSod[index] = makeSpin(1.0, 1.0e6, 700.0, 10.0);
        m_drrSod[index]->setDecimals(1);
        m_drrSod[index]->setSuffix(" mm");
        m_drrDetectorWidth[index] = makeSpin(32.0, 4096.0, 320.0, 10.0);
        m_drrDetectorWidth[index]->setDecimals(1);
        m_drrDetectorWidth[index]->setSuffix(" mm");
        m_drrDetectorHeight[index] = makeSpin(32.0, 4096.0, 240.0, 10.0);
        m_drrDetectorHeight[index]->setDecimals(1);
        m_drrDetectorHeight[index]->setSuffix(" mm");
        m_drrPixelSpacing[index] = makeSpin(0.0, 1000.0, 0.0, 0.05);
        m_drrPixelSpacing[index]->setDecimals(3);
        m_drrPixelSpacing[index]->setSuffix(" mm");
        m_drrPixelSpacing[index]->setSpecialValueText("Auto");
        m_drrRayStep[index] = makeSpin(0.001, 1000.0, 1.0, 0.1);
        m_drrRayStep[index]->setDecimals(3);
        m_drrRayStep[index]->setSuffix(" mm");
        m_drrWindowCenter[index] = makeSpin(0.0, 1.0e6, 0.0, 10.0);
        m_drrWindowCenter[index]->setDecimals(2);
        m_drrWindowCenter[index]->setSpecialValueText("Auto");
        m_drrWindowWidth[index] = makeSpin(0.0, 1.0e6, 0.0, 10.0);
        m_drrWindowWidth[index]->setDecimals(2);
        m_drrWindowWidth[index]->setSpecialValueText("Auto");
        m_drrGamma[index] = makeSpin(0.1, 10.0, 1.0, 0.1);
        m_drrGamma[index]->setDecimals(2);
        m_drrHuOffset[index] = makeSpin(-10000.0, 10000.0, 0.0, 10.0);
        m_drrHuOffset[index]->setDecimals(1);
        m_drrHuScale[index] = makeSpin(0.001, 100.0, 1.0, 0.05);
        m_drrHuScale[index]->setDecimals(4);

        drrForm->addRow("SID", m_drrSid[index]);
        drrForm->addRow("SOD", m_drrSod[index]);
        drrForm->addRow("Detector width", m_drrDetectorWidth[index]);
        drrForm->addRow("Detector height", m_drrDetectorHeight[index]);
        drrForm->addRow("Pixel spacing", m_drrPixelSpacing[index]);
        drrForm->addRow("Ray step", m_drrRayStep[index]);
        drrForm->addRow("Window center", m_drrWindowCenter[index]);
        drrForm->addRow("Window width", m_drrWindowWidth[index]);
        drrForm->addRow("Gamma", m_drrGamma[index]);
        drrForm->addRow("HU offset", m_drrHuOffset[index]);
        drrForm->addRow("HU scale", m_drrHuScale[index]);
        drrTabs->addTab(page, title);
    };
    makeDrrParameterPage(measurement::XrayPreset::AP, "AP");
    makeDrrParameterPage(measurement::XrayPreset::LAT, "LAT");
    drrGroupLayout->addWidget(drrTabs);
    drrPanelLayout->addWidget(drrGroup);
    drrPanelLayout->addStretch(1);
    drrPanel->setMinimumWidth(280);

    splitter->addWidget(mprPanel);
    splitter->addWidget(xrayPanel);
    splitter->addWidget(controlPanel);
    splitter->addWidget(drrPanel);
    splitter->setChildrenCollapsible(false);
    // The planning column is reserved for measurement and instrument workflows;
    // the rightmost auxiliary column owns project, status, patient, and DRR
    // controls.
    splitter->setStretchFactor(0, 4);
    splitter->setStretchFactor(1, 2);
    splitter->setStretchFactor(2, 2);
    splitter->setStretchFactor(3, 2);
    splitter->setSizes({620, 320, 340, 300});

    const auto crosshairCallback = [this](measurement::Vec3d patient) {
        setCrosshairPatient(patient);
    };
    const auto windowLevelCallback = [this](double centerHu, double widthHu) {
        setWindowLevel(centerHu, widthHu);
    };
    const auto rotationCallback = [this](measurement::MprPlane aroundPlane, measurement::MprPlane linePlane, double deltaAngleRad) {
        rotateCrosshairPlane(aroundPlane, linePlane, deltaAngleRad);
    };
    m_axialView->setCrosshairChangedCallback(crosshairCallback);
    m_sagittalView->setCrosshairChangedCallback(crosshairCallback);
    m_coronalView->setCrosshairChangedCallback(crosshairCallback);
    m_axialView->setWindowLevelChangedCallback(windowLevelCallback);
    m_sagittalView->setWindowLevelChangedCallback(windowLevelCallback);
    m_coronalView->setWindowLevelChangedCallback(windowLevelCallback);
    m_axialView->setPlaneRotationCallback(rotationCallback);
    m_sagittalView->setPlaneRotationCallback(rotationCallback);
    m_coronalView->setPlaneRotationCallback(rotationCallback);
    const auto activatePlaneCallback = [this](measurement::MprPlane plane) {
        activateMprPlane(plane);
    };
    m_axialView->setActivatedCallback(activatePlaneCallback);
    m_sagittalView->setActivatedCallback(activatePlaneCallback);
    m_coronalView->setActivatedCallback(activatePlaneCallback);
    m_axialView->setLinkedPlaneFrames(&m_planeFrames);
    m_sagittalView->setLinkedPlaneFrames(&m_planeFrames);
    m_coronalView->setLinkedPlaneFrames(&m_planeFrames);
    m_axialView->setMeasurements(&m_measurementStore.all());
    m_sagittalView->setMeasurements(&m_measurementStore.all());
    m_coronalView->setMeasurements(&m_measurementStore.all());

    const auto measurementPointCallback = [this](
                                              measurement::MprPlane plane,
                                              measurement::Vec3d patientPoint,
                                              measurement::MeasurementPlane slicePlane) {
        handleMeasurementPointAdded(plane, patientPoint, slicePlane);
    };
    const auto measurementHoverCallback = [this](measurement::MprPlane plane, std::optional<measurement::Vec3d> patientPoint) {
        handleMeasurementHoverChanged(plane, patientPoint);
    };
    const auto measurementCancelCallback = [this]() {
        cancelPendingMeasurement();
    };
    m_axialView->setMeasurementPointAddedCallback(measurementPointCallback);
    m_sagittalView->setMeasurementPointAddedCallback(measurementPointCallback);
    m_coronalView->setMeasurementPointAddedCallback(measurementPointCallback);
    m_axialView->setMeasurementHoverChangedCallback(measurementHoverCallback);
    m_sagittalView->setMeasurementHoverChangedCallback(measurementHoverCallback);
    m_coronalView->setMeasurementHoverChangedCallback(measurementHoverCallback);
    m_axialView->setMeasurementCancelCallback(measurementCancelCallback);
    m_sagittalView->setMeasurementCancelCallback(measurementCancelCallback);
    m_coronalView->setMeasurementCancelCallback(measurementCancelCallback);

    const auto drrLineCompleted = [this](measurement::XrayPreset preset, DrrDetectorLine line) {
        handleDrrLineCompleted(preset, line);
    };
    const auto drrInstrumentSelected = [this](std::string id) {
        handleDrrInstrumentSelected(std::move(id));
    };
    const auto drrInstrumentDragged = [this](
                                          measurement::XrayPreset preset,
                                          std::string id,
                                          DrrInteractionTarget target,
                                          DrrDetectorPoint detectorPoint) {
        handleDrrInstrumentDragged(preset, std::move(id), target, detectorPoint);
    };
    m_apXrayView->setLineCompletedCallback(drrLineCompleted);
    m_latXrayView->setLineCompletedCallback(drrLineCompleted);
    m_apXrayView->setInstrumentSelectedCallback(drrInstrumentSelected);
    m_latXrayView->setInstrumentSelectedCallback(drrInstrumentSelected);
    m_apXrayView->setInstrumentDraggedCallback(drrInstrumentDragged);
    m_latXrayView->setInstrumentDraggedCallback(drrInstrumentDragged);

    connect(m_loadDicomButton, &QPushButton::clicked, this, [this]() { loadDicomFolder(); });
    connect(resetViewsButton, &QPushButton::clicked, this, [this]() { resetAllViews(); });
    connect(saveButton, &QPushButton::clicked, this, [this]() { saveProject(); });
    connect(m_freeObliqueButton, &QPushButton::toggled, this, [this](bool checked) { setFreeObliqueMode(checked); });
    connect(m_measureNavigateButton, &QPushButton::clicked, this, [this]() { setMeasurementMode(measurement::MeasurementMode::Navigate); });
    connect(m_measureDistanceButton, &QPushButton::clicked, this, [this]() { setMeasurementMode(measurement::MeasurementMode::Distance); });
    connect(m_measureAngleButton, &QPushButton::clicked, this, [this]() { setMeasurementMode(measurement::MeasurementMode::Angle); });
    connect(deleteMeasurement, &QPushButton::clicked, this, [this]() { deleteSelectedMeasurement(); });
    connect(clearMeasurementsButton, &QPushButton::clicked, this, [this]() { clearMeasurements(); });
    connect(m_measurementLabel, &QLineEdit::editingFinished, this, [this]() { renameSelectedMeasurement(); });
    const auto measurementSelectionChanged = [this]() {
        if (const auto id = selectedMeasurementId()) {
            selectMeasurementById(*id);
            jumpToMeasurement(*id);
        }
    };
    connect(m_measurementList, &QListWidget::currentRowChanged, this, [measurementSelectionChanged](int) {
        measurementSelectionChanged();
    });
    connect(m_measurementList, &QListWidget::itemClicked, this, [measurementSelectionChanged](QListWidgetItem*) {
        measurementSelectionChanged();
    });
    connect(addPin, &QPushButton::clicked, this, [this]() { addInstrument(measurement::InstrumentType::GuidePin); });
    connect(addScrew, &QPushButton::clicked, this, [this]() { addInstrument(measurement::InstrumentType::PedicleScrew); });
    connect(m_editInstrumentButton, &QPushButton::clicked, this, [this]() { toggleInstrumentEdit(); });
    connect(remove, &QPushButton::clicked, this, [this]() { removeSelectedInstrument(); });
    connect(m_drrPinButton, &QPushButton::clicked, this, [this]() {
        setDrrPlacementMode(measurement::InstrumentType::GuidePin);
    });
    connect(m_drrScrewButton, &QPushButton::clicked, this, [this]() {
        setDrrPlacementMode(measurement::InstrumentType::PedicleScrew);
    });
    connect(m_drrCancelButton, &QPushButton::clicked, this, [this]() { cancelDrrPlacement(); });
    const auto instrumentSelectionChanged = [this]() {
        syncPlacementSelectionFromUi();
        const bool jumped = jumpToInstrumentPlanningPose(selectedInstrumentId());
        syncSpinBoxesFromSelectedInstrument();
        if (!jumped) {
            refreshAll(true);
        }
    };
    connect(m_instrumentList, &QListWidget::currentRowChanged, this, [instrumentSelectionChanged]() {
        instrumentSelectionChanged();
    });
    connect(m_instrumentList, &QListWidget::itemClicked, this, [instrumentSelectionChanged](QListWidgetItem*) {
        instrumentSelectionChanged();
    });
    connect(m_patientPostureCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) {
        const bool prone = m_patientPostureCombo != nullptr
            && m_patientPostureCombo->currentData().toBool();
        const bool feetFirst = m_headFeetDirectionCombo != nullptr
            && m_headFeetDirectionCombo->currentData().toBool();
        applyPatientPosition(prone, feetFirst);
    });
    connect(m_headFeetDirectionCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) {
        const bool prone = m_patientPostureCombo != nullptr
            && m_patientPostureCombo->currentData().toBool();
        const bool feetFirst = m_headFeetDirectionCombo != nullptr
            && m_headFeetDirectionCombo->currentData().toBool();
        applyPatientPosition(prone, feetFirst);
    });

    const auto sliderChanged = [this]() {
        if (m_syncingControls) {
            return;
        }
        setCrosshairVoxel({
            static_cast<double>(m_xSlider->value()),
            static_cast<double>(m_ySlider->value()),
            static_cast<double>(m_zSlider->value()),
        });
    };
    connect(m_xSlider, &QSlider::valueChanged, this, sliderChanged);
    connect(m_ySlider, &QSlider::valueChanged, this, sliderChanged);
    connect(m_zSlider, &QSlider::valueChanged, this, sliderChanged);
    connect(m_label, &QLineEdit::editingFinished, this, [this]() { applyInstrumentPropertyEdits(); });
    connect(m_length, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { applyInstrumentPropertyEdits(); });
    connect(m_diameter, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { applyInstrumentPropertyEdits(); });

    const auto drrSettingChanged = [this](double) {
        QObject* changedControl = sender();
        bool displayOnly = false;
        for (size_t index = 0; index < m_drrWindowCenter.size(); ++index) {
            displayOnly = displayOnly
                || changedControl == m_drrWindowCenter[index]
                || changedControl == m_drrWindowWidth[index]
                || changedControl == m_drrGamma[index];
        }
        for (size_t index = 0; index < m_drrSid.size(); ++index) {
            if (m_drrSid[index] != nullptr && m_drrSod[index] != nullptr && m_drrSod[index]->value() >= m_drrSid[index]->value()) {
                m_drrSod[index]->blockSignals(true);
                m_drrSod[index]->setValue(std::max(1.0, m_drrSid[index]->value() - 1.0));
                m_drrSod[index]->blockSignals(false);
            }
        }
        if (displayOnly) {
            if (m_apXrayView != nullptr) {
                m_apXrayView->setDrrSettings(drrSettingsFromControls(measurement::XrayPreset::AP));
                m_apXrayView->refreshDisplaySettings();
            }
            if (m_latXrayView != nullptr) {
                m_latXrayView->setDrrSettings(drrSettingsFromControls(measurement::XrayPreset::LAT));
                m_latXrayView->refreshDisplaySettings();
            }
            if (m_sceneView != nullptr) {
                std::array<QImage, 2> drrImages{};
                if (m_apXrayView != nullptr) {
                    drrImages[0] = m_apXrayView->renderedImage();
                }
                if (m_latXrayView != nullptr) {
                    drrImages[1] = m_latXrayView->renderedImage();
                }
                m_sceneView->setDrrImages(std::move(drrImages));
                m_sceneView->refreshScene();
            }
            return;
        }
        refreshXrayViews();
    };
    for (size_t index = 0; index < m_drrSid.size(); ++index) {
        connect(m_drrSid[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrSod[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrDetectorWidth[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrDetectorHeight[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrPixelSpacing[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrRayStep[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrWindowCenter[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrWindowWidth[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrGamma[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrHuOffset[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrHuScale[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
    }

    setMeasurementMode(measurement::MeasurementMode::Navigate);
    refreshMeasurementList();
    resize(1560, 840);
}

void MprPlanVerificationWindow::loadSyntheticVolume()
{
    m_volume = makeSyntheticVolume();
    if (m_loadDicomButton != nullptr) {
        m_loadDicomButton->setEnabled(true);
    }
    activateLoadedVolumeData();
}

void MprPlanVerificationWindow::loadStartupVolume()
{
    // Startup should be fast and deterministic: show the built-in phantom first
    // and leave DICOM loading as an explicit user action.
    loadSyntheticVolume();
    statusBar()->showMessage("Loaded synthetic phantom.", 6000);
}

void MprPlanVerificationWindow::loadDicomFolder()
{
    QString initialDirectory = QString::fromStdString(m_volume.sourceFolder);
    if (!QFileInfo(initialDirectory).isDir()) {
        initialDirectory = QDir::homePath();
    }
    const QString folder = QFileDialog::getExistingDirectory(
        this,
        "Load CT DICOM folder",
        initialDirectory);
    if (folder.isEmpty()) {
        return;
    }

    QString failureMessage;
    if (!tryLoadDicomFolder(folder, &failureMessage)) {
        statusBar()->showMessage(failureMessage, 8000);
        QMessageBox::warning(this, "Load DICOM failed", failureMessage);
        return;
    }

    if (m_loadDicomButton != nullptr) {
        m_loadDicomButton->setEnabled(false);
    }
    statusBar()->showMessage(QString("Loaded DICOM: %1").arg(QDir::toNativeSeparators(folder)), 6000);
}

bool MprPlanVerificationWindow::tryLoadDicomFolder(const QString& folder, QString* failureMessage)
{
    ScopedModalBusyDialog busyDialog(
        this,
        QString("Loading DICOM from %1...").arg(QDir::toNativeSeparators(folder)),
        "Loading DICOM");
    measurement::DicomVolumeLoader loader;
    const auto loaded = loader.loadFolder(folder.toStdString());
    if (!loaded.ok()) {
        if (failureMessage != nullptr) {
            *failureMessage = summarizeLoadFailure(QDir::toNativeSeparators(folder), loaded.error());
        }
        return false;
    }

    m_volume = loaded.value();
    activateLoadedVolumeData();
    return true;
}

void MprPlanVerificationWindow::activateLoadedVolumeData()
{
    // DRR placement lines and active drags are detector-space state from the
    // previous volume. Drop them before publishing the new patient geometry.
    m_drrPlacementType.reset();
    m_pendingDrrLines = {};
    if (m_drrPinButton != nullptr) {
        const bool blocked = m_drrPinButton->blockSignals(true);
        m_drrPinButton->setChecked(false);
        m_drrPinButton->blockSignals(blocked);
    }
    if (m_drrScrewButton != nullptr) {
        const bool blocked = m_drrScrewButton->blockSignals(true);
        m_drrScrewButton->setChecked(false);
        m_drrScrewButton->blockSignals(blocked);
    }

    resetPatientPositionControls();
    initializePlaneFrames();
    resetCrosshairToVolumeCenter(false);
    syncPlaneFrameOrigins();
    syncSlidersFromCrosshair();
    syncPerViewStates();

    const std::string selectedId = selectedInstrumentId();
    const std::array<IMprSliceView*, 3> views{m_axialView, m_sagittalView, m_coronalView};
    for (size_t index = 0; index < views.size(); ++index) {
        IMprSliceView* view = views[index];
        if (view == nullptr) {
            continue;
        }
        view->setVolume(&m_volume);
        view->setLinkedPlaneFrames(&m_planeFrames);
        view->setState(&m_viewStates[index]);
        view->setPlan(&m_plan);
        view->setSelectedInstrumentId(selectedId);
    }

    if (m_sceneView != nullptr) {
        m_sceneView->setVolume(&m_volume);
        m_sceneView->setPlan(&m_plan);
        m_sceneView->setSelectedInstrumentId(selectedId);
        m_sceneView->resetCamera();
    }

    const std::array<std::optional<DrrDetectorLine>, 2> noConstraints{};
    if (m_apXrayView != nullptr) {
        m_apXrayView->setVolume(&m_volume);
        m_apXrayView->setPlan(&m_plan);
        m_apXrayView->setSelectedInstrumentId(selectedId);
        m_apXrayView->setDrrSettings(drrSettingsFromControls(measurement::XrayPreset::AP));
        m_apXrayView->setPlacementActive(false);
        m_apXrayView->setPendingLine(std::nullopt);
        m_apXrayView->setPlacementConstraints(noConstraints);
    }
    if (m_latXrayView != nullptr) {
        m_latXrayView->setVolume(&m_volume);
        m_latXrayView->setPlan(&m_plan);
        m_latXrayView->setSelectedInstrumentId(selectedId);
        m_latXrayView->setDrrSettings(drrSettingsFromControls(measurement::XrayPreset::LAT));
        m_latXrayView->setPlacementActive(false);
        m_latXrayView->setPendingLine(std::nullopt);
        m_latXrayView->setPlacementConstraints(noConstraints);
    }

    refreshAll(true);
}

void MprPlanVerificationWindow::saveProject()
{
    const QString path = QFileDialog::getSaveFileName(this, "Save project", {}, "MPR Project (*.mprproj)");
    if (path.isEmpty()) {
        return;
    }
    const auto result = measurement::saveProjectFile(makeManifest(), path.toStdString());
    if (!result.ok()) {
        statusBar()->showMessage(QString::fromStdString(result.error().message + ": " + result.error().detail), 8000);
        return;
    }
    statusBar()->showMessage("Project saved", 4000);
}

void MprPlanVerificationWindow::resetCrosshairToVolumeCenter(bool refreshViews)
{
    const measurement::Size3i dims = m_volume.metadata.dimensions;
    setCrosshairVoxel({
        static_cast<double>(dims.x - 1) * 0.5,
        static_cast<double>(dims.y - 1) * 0.5,
        static_cast<double>(dims.z - 1) * 0.5,
    }, refreshViews);
}

void MprPlanVerificationWindow::setCrosshairVoxel(measurement::Vec3d voxel, bool refreshViews)
{
    if (!m_volume.image || !isFiniteVec(voxel)) {
        return;
    }
    const measurement::Size3i dims = m_volume.metadata.dimensions;
    if (dims.x <= 0 || dims.y <= 0 || dims.z <= 0) {
        return;
    }
    voxel.x = clampDouble(voxel.x, 0.0, static_cast<double>(dims.x - 1));
    voxel.y = clampDouble(voxel.y, 0.0, static_cast<double>(dims.y - 1));
    voxel.z = clampDouble(voxel.z, 0.0, static_cast<double>(dims.z - 1));
    m_mprState.crosshairPatientMm = measurement::voxelToPatient(m_volume.transform, voxel);
    if (!isFiniteVec(m_mprState.crosshairPatientMm)) {
        return;
    }
    if (!refreshViews) {
        return;
    }
    const auto placementResult = m_placementController->onCrosshairChanged(m_mprState.crosshairPatientMm);
    if (!placementResult.ok()) {
        statusBar()->showMessage(QString::fromStdString(placementResult.error().message), 6000);
    }
    syncPlaneFrameOrigins();
    syncSlidersFromCrosshair();
    if (m_placementController->editMode() == measurement::InstrumentPlacementEditMode::BindEntryToCrosshair) {
        syncSpinBoxesFromSelectedInstrument();
    }
    refreshAll(m_placementController->editMode() == measurement::InstrumentPlacementEditMode::BindEntryToCrosshair);
}

void MprPlanVerificationWindow::setCrosshairPatient(measurement::Vec3d patient)
{
    if (!m_volume.image || !isFiniteVec(patient)) {
        return;
    }
    const measurement::Vec3d voxel = measurement::patientToVoxel(m_volume.transform, patient);
    if (!isFiniteVec(voxel)) {
        return;
    }
    setCrosshairVoxel(voxel);
}

void MprPlanVerificationWindow::setWindowLevel(double centerHu, double widthHu)
{
    m_mprState.windowCenterHu = centerHu;
    m_mprState.windowWidthHu = std::max(widthHu, 1.0);
    refreshAll();
}

void MprPlanVerificationWindow::resetAllViews()
{
    for (MprSliceWidget* view : {m_axialView, m_sagittalView, m_coronalView}) {
        view->resetViewPresentation();
    }
    if (m_sceneView != nullptr) {
        m_sceneView->resetCamera();
    }
    refreshAll();
}

void MprPlanVerificationWindow::resetPatientPositionControls()
{
    const std::string code = m_volume.patientPositionCode.empty() ? "HFS" : m_volume.patientPositionCode;
    const bool prone = code == "HFP" || code == "FFP";
    const bool feetFirst = code == "FFS" || code == "FFP";
    m_appliedPatientProne = prone;
    m_appliedFeetFirst = feetFirst;
    if (m_patientPostureCombo != nullptr) {
        m_patientPostureCombo->blockSignals(true);
        m_patientPostureCombo->setCurrentIndex(prone ? 1 : 0);
        m_patientPostureCombo->blockSignals(false);
    }
    if (m_headFeetDirectionCombo != nullptr) {
        m_headFeetDirectionCombo->blockSignals(true);
        m_headFeetDirectionCombo->setCurrentIndex(feetFirst ? 1 : 0);
        m_headFeetDirectionCombo->blockSignals(false);
    }
}

void MprPlanVerificationWindow::applyPatientPosition(bool prone, bool feetFirst)
{
    if (!m_volume.image) {
        m_appliedPatientProne = prone;
        m_appliedFeetFirst = feetFirst;
        return;
    }

    const bool postureChanged = prone != m_appliedPatientProne;
    const bool directionChanged = feetFirst != m_appliedFeetFirst;
    if (!postureChanged && !directionChanged) {
        return;
    }

    const measurement::Vec3d center = volumeCenterPatient(m_volume);
    if (!isFiniteVec(center)) {
        return;
    }

    ScopedModalBusyDialog busyDialog(this, "Updating patient position...", "Updating Patient Position");

    const auto applyRigidRotation = [&](measurement::Vec3d axis, double angleRad) {
        const auto transformPoint = [&](measurement::Vec3d point) {
            return rotatePointAroundAxis(point, center, axis, angleRad);
        };
        const auto transformDirection = [&](measurement::Vec3d direction) {
            return measurement::normalize(rotateAroundAxis(direction, axis, angleRad));
        };

        measurement::VolumeMetadata updatedMetadata = m_volume.metadata;
        updatedMetadata.originPatientMm = transformPoint(updatedMetadata.originPatientMm);
        updatedMetadata.rowDirectionPatient = transformDirection(updatedMetadata.rowDirectionPatient);
        updatedMetadata.columnDirectionPatient = transformDirection(updatedMetadata.columnDirectionPatient);
        updatedMetadata.sliceDirectionPatient = transformDirection(updatedMetadata.sliceDirectionPatient);

        const auto updatedTransform = measurement::makeVolumeTransform(updatedMetadata);
        if (!updatedTransform.ok()) {
            statusBar()->showMessage(QString::fromStdString(updatedTransform.error().message), 6000);
            return false;
        }

        measurement::VolumeData updatedVolume = m_volume;
        updatedVolume.metadata = updatedMetadata;
        updatedVolume.transform = updatedTransform.value();

        measurement::SurgicalPlan transformedPlan;
        for (const measurement::Instrument& instrument : m_plan.instruments()) {
            measurement::Instrument transformed = instrument;
            transformed.entryPointPatientMm = transformPoint(instrument.entryPointPatientMm);
            transformed.directionPatientUnit = transformDirection(instrument.directionPatientUnit);
            const auto addResult = transformedPlan.addInstrument(std::move(transformed));
            if (!addResult.ok()) {
                statusBar()->showMessage(QString::fromStdString(addResult.error().message), 6000);
                return false;
            }
        }

        m_volume = std::move(updatedVolume);
        m_plan = std::move(transformedPlan);
        m_mprState.crosshairPatientMm = transformPoint(m_mprState.crosshairPatientMm);
        m_editOriginalPatch.entryPointPatientMm = transformPoint(m_editOriginalPatch.entryPointPatientMm);
        m_editOriginalPatch.directionPatientUnit = transformDirection(m_editOriginalPatch.directionPatientUnit);
        return true;
    };

    if (postureChanged) {
        const measurement::Vec3d axis = measurement::normalize(m_volume.metadata.sliceDirectionPatient);
        if (!isFiniteVec(axis) || measurement::length(axis) <= 1.0e-6 || !applyRigidRotation(axis, kPi)) {
            return;
        }
    }
    if (directionChanged) {
        const measurement::Vec3d axis = measurement::normalize(m_volume.metadata.columnDirectionPatient);
        if (!isFiniteVec(axis) || measurement::length(axis) <= 1.0e-6 || !applyRigidRotation(axis, kPi)) {
            return;
        }
    }

    m_pendingDrrLines = {};
    m_appliedPatientProne = prone;
    m_appliedFeetFirst = feetFirst;
    m_volume.patientPositionCode = feetFirst
        ? (prone ? "FFP" : "FFS")
        : (prone ? "HFP" : "HFS");
    initializePlaneFrames();
    syncSlidersFromCrosshair();
    refreshAll(true);
}

void MprPlanVerificationWindow::initializePlaneFrames()
{
    const std::array<measurement::MprPlane, 3> planes{
        measurement::MprPlane::Axial,
        measurement::MprPlane::Sagittal,
        measurement::MprPlane::Coronal,
    };
    for (size_t index = 0; index < planes.size(); ++index) {
        const auto frame = measurement::defaultSliceFrame(
            m_volume.metadata,
            planes[index],
            m_mprState.crosshairPatientMm);
        if (frame.ok()) {
            m_planeFrames[index] = normalizedFrame(frame.value());
        }
    }
}

void MprPlanVerificationWindow::syncPlaneFrameOrigins()
{
    for (measurement::MprSliceFrame& frame : m_planeFrames) {
        frame.originPatientMm = m_mprState.crosshairPatientMm;
    }
}

void MprPlanVerificationWindow::syncPerViewStates()
{
    const std::array<measurement::MprPlane, 3> planes{
        measurement::MprPlane::Axial,
        measurement::MprPlane::Sagittal,
        measurement::MprPlane::Coronal,
    };
    for (size_t index = 0; index < planes.size(); ++index) {
        m_viewStates[index] = m_mprState;
        m_viewStates[index].plane = planes[index];
        m_viewStates[index].obliqueFrame = normalizedFrame(m_planeFrames[index]);
    }
}

void MprPlanVerificationWindow::rotateCrosshairPlane(
    measurement::MprPlane aroundPlane,
    measurement::MprPlane linePlane,
    double deltaAngleRad)
{
    if (!std::isfinite(deltaAngleRad) || std::abs(deltaAngleRad) <= 1.0e-6 || aroundPlane == linePlane) {
        return;
    }

    m_activeMprPlane = aroundPlane;
    m_activeCrosshairLinePlane = linePlane;
    const int aroundIndex = planeIndex(aroundPlane);
    const measurement::Vec3d axis = measurement::normalize(m_planeFrames[aroundIndex].normalPatientUnit);
    if (!isFiniteVec(axis) || measurement::length(axis) <= 1.0e-6) {
        return;
    }

    if (m_freeObliqueMode) {
        const int lineIndex = planeIndex(linePlane);
        measurement::MprSliceFrame rotated = m_planeFrames[lineIndex];
        rotated.horizontalPatientUnit = rotateAroundAxis(rotated.horizontalPatientUnit, axis, deltaAngleRad);
        rotated.verticalPatientUnit = rotateAroundAxis(rotated.verticalPatientUnit, axis, deltaAngleRad);
        rotated.normalPatientUnit = rotateAroundAxis(rotated.normalPatientUnit, axis, deltaAngleRad);
        rotated.originPatientMm = m_mprState.crosshairPatientMm;
        m_planeFrames[lineIndex] = normalizedFrame(rotated);
    } else {
        for (int index = 0; index < static_cast<int>(m_planeFrames.size()); ++index) {
            if (index == aroundIndex) {
                continue;
            }

            measurement::MprSliceFrame rotated = m_planeFrames[index];
            rotated.horizontalPatientUnit = rotateAroundAxis(rotated.horizontalPatientUnit, axis, deltaAngleRad);
            rotated.verticalPatientUnit = rotateAroundAxis(rotated.verticalPatientUnit, axis, deltaAngleRad);
            rotated.normalPatientUnit = rotateAroundAxis(rotated.normalPatientUnit, axis, deltaAngleRad);
            rotated.originPatientMm = m_mprState.crosshairPatientMm;
            m_planeFrames[index] = normalizedFrame(rotated);
        }

        orthogonalizePlaneFrames(aroundPlane, linePlane);
    }

    m_planeFrames[aroundIndex] = normalizedFrame(m_planeFrames[aroundIndex]);
    syncPlaneFrameOrigins();
    alignEditingInstrumentToCrosshairLine(aroundPlane, linePlane);
    refreshAll(m_instrumentEditActive);
}

void MprPlanVerificationWindow::setFreeObliqueMode(bool enabled)
{
    if (m_freeObliqueMode == enabled) {
        if (m_freeObliqueButton != nullptr) {
            m_freeObliqueButton->setText(enabled ? "Free oblique: On" : "Free oblique: Off");
            if (m_freeObliqueButton->isChecked() != enabled) {
                m_freeObliqueButton->setChecked(enabled);
            }
        }
        return;
    }

    m_freeObliqueMode = enabled;
    if (m_freeObliqueButton != nullptr) {
        m_freeObliqueButton->setText(enabled ? "Free oblique: On" : "Free oblique: Off");
        if (m_freeObliqueButton->isChecked() != enabled) {
            m_freeObliqueButton->setChecked(enabled);
        }
    }

    if (!enabled) {
        orthogonalizePlaneFrames(m_activeMprPlane, m_activeCrosshairLinePlane);
        alignEditingInstrumentToCrosshairLine(m_activeMprPlane, m_activeCrosshairLinePlane);
    }
    statusBar()->showMessage(enabled ? "Free oblique mode enabled" : "Orthogonal MPR mode enabled", 4000);
    refreshAll(m_instrumentEditActive);
}

void MprPlanVerificationWindow::orthogonalizePlaneFrames(
    measurement::MprPlane anchorPlane,
    measurement::MprPlane linePlane)
{
    if (anchorPlane == linePlane) {
        linePlane = anchorPlane == measurement::MprPlane::Axial
            ? measurement::MprPlane::Sagittal
            : measurement::MprPlane::Axial;
    }

    const measurement::MprPlane otherPlane = remainingPlane(anchorPlane, linePlane);
    const int anchorIndex = planeIndex(anchorPlane);
    const int lineIndex = planeIndex(linePlane);
    const int otherIndex = planeIndex(otherPlane);

    const measurement::Vec3d anchorNormal = measurement::normalize(m_planeFrames[anchorIndex].normalPatientUnit);
    if (!isFiniteVec(anchorNormal) || measurement::length(anchorNormal) <= 1.0e-6) {
        return;
    }

    measurement::Vec3d lineNormal = projectedOntoPlane(m_planeFrames[lineIndex].normalPatientUnit, anchorNormal);
    if (measurement::length(lineNormal) <= 1.0e-6) {
        lineNormal = projectedOntoPlane(m_planeFrames[anchorIndex].horizontalPatientUnit, anchorNormal);
    }
    if (measurement::length(lineNormal) <= 1.0e-6) {
        lineNormal = fallbackInPlaneAxis(anchorNormal);
    }
    lineNormal = measurement::normalize(lineNormal);
    if (measurement::dot(lineNormal, m_planeFrames[lineIndex].normalPatientUnit) < 0.0) {
        lineNormal = lineNormal * -1.0;
    }

    measurement::Vec3d otherNormal = measurement::normalize(measurement::cross(anchorNormal, lineNormal));
    if (measurement::dot(otherNormal, m_planeFrames[otherIndex].normalPatientUnit) < 0.0) {
        otherNormal = otherNormal * -1.0;
    }

    m_planeFrames[anchorIndex] = frameWithNormal(m_planeFrames[anchorIndex], anchorNormal);
    m_planeFrames[lineIndex] = frameWithNormal(m_planeFrames[lineIndex], lineNormal);
    m_planeFrames[otherIndex] = frameWithNormal(m_planeFrames[otherIndex], otherNormal);
    syncPlaneFrameOrigins();
}

void MprPlanVerificationWindow::syncSlidersFromCrosshair()
{
    if (!m_volume.image) {
        return;
    }

    const measurement::Size3i dims = m_volume.metadata.dimensions;
    const measurement::Vec3d voxel = measurement::patientToVoxel(m_volume.transform, m_mprState.crosshairPatientMm);
    m_syncingControls = true;
    m_xSlider->setRange(0, dims.x - 1);
    m_ySlider->setRange(0, dims.y - 1);
    m_zSlider->setRange(0, dims.z - 1);
    m_xSlider->setValue(clampIndex(voxel.x, dims.x));
    m_ySlider->setValue(clampIndex(voxel.y, dims.y));
    m_zSlider->setValue(clampIndex(voxel.z, dims.z));
    m_xValueLabel->setText(QString::number(m_xSlider->value()));
    m_yValueLabel->setText(QString::number(m_ySlider->value()));
    m_zValueLabel->setText(QString::number(m_zSlider->value()));
    m_syncingControls = false;
}

void MprPlanVerificationWindow::refreshAll(bool refreshScene)
{
    syncPlaneFrameOrigins();
    syncPerViewStates();

    const std::string selectedId = selectedInstrumentId();
    const std::array<IMprSliceView*, 3> views{m_axialView, m_sagittalView, m_coronalView};
    for (size_t index = 0; index < views.size(); ++index) {
        IMprSliceView* view = views[index];
        view->setVolume(&m_volume);
        view->setLinkedPlaneFrames(&m_planeFrames);
        view->setState(&m_viewStates[index]);
        view->setPlan(&m_plan);
        view->setSelectedInstrumentId(selectedId);
        view->refreshImage();
    }
    if (refreshScene) {
        refreshPlanScene();
    }
    refreshXrayViews();
    refreshInstrumentList();
    refreshMeasurementOverlays();
    refreshMeasurementList();
    refreshStatus();
}

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
        statusBar()->showMessage("Measurement added.", 3000);
    } else {
        m_measurementHoverPatientMm = patientPoint;
        const size_t requiredPointCount = m_measurementMode == measurement::MeasurementMode::Angle ? 4U : 2U;
        if (previousPointCount + 1U >= requiredPointCount && m_measurementStateMachine.pendingPoints().empty()) {
            statusBar()->showMessage("Measurement was rejected because the picked points are degenerate.", 5000);
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
        statusBar()->showMessage("Selected measurement cannot be located.", 4000);
        return;
    }

    if (const auto plane = planeForMeasurementViewType(annotation->createdViewType)) {
        activateMprPlane(*plane);
    }
    setCrosshairPatient(*anchor);
    statusBar()->showMessage(QString("Measurement #%1 located.").arg(id.value()), 3000);
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

void MprPlanVerificationWindow::refreshPlanScene()
{
    if (m_sceneView == nullptr) {
        return;
    }

    m_sceneView->setVolume(&m_volume);
    m_sceneView->setPlan(&m_plan);
    m_sceneView->setSelectedInstrumentId(selectedInstrumentId());
    m_sceneView->refreshScene();
}

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

void MprPlanVerificationWindow::selectInstrumentById(const std::string& id)
{
    if (id.empty() || m_instrumentList == nullptr) {
        return;
    }
    m_instrumentList->blockSignals(true);
    for (int row = 0; row < m_instrumentList->count(); ++row) {
        if (m_instrumentList->item(row)->data(Qt::UserRole).toString().toStdString() == id) {
            m_instrumentList->setCurrentRow(row);
            break;
        }
    }
    m_instrumentList->blockSignals(false);
    m_placementController->setSelectedInstrumentId(id);
}

void MprPlanVerificationWindow::refreshStatus()
{
    const measurement::Size3i dims = m_volume.metadata.dimensions;
    m_volumeLabel->setText(QString("Volume: %1 x %2 x %3, spacing %4, HU [%5, %6]\nPatient position: %7\nSource: %8")
                               .arg(dims.x)
                               .arg(dims.y)
                               .arg(dims.z)
                               .arg(vecText(m_volume.metadata.spacingMm))
                               .arg(m_volume.metadata.minHu)
                               .arg(m_volume.metadata.maxHu)
                               .arg(QString::fromStdString(m_volume.patientPositionCode))
                               .arg(QString::fromStdString(m_volume.sourceFolder)));

    const QString editMode = m_instrumentEditActive
        ? QString("editing %1").arg(QString::fromStdString(m_editingInstrumentId))
        : QString("off");
    QString drrPlacement = "off";
    if (m_drrPlacementType.has_value()) {
        drrPlacement = *m_drrPlacementType == measurement::InstrumentType::GuidePin ? "Guide pin" : "Pedicle screw";
        drrPlacement += m_pendingDrrLines[0].has_value()
            ? " (AP fixed, draw constrained LAT)"
            : " (draw AP first)";
    }
    const QString pendingMeasurement = m_pendingMeasurementPlane.has_value()
        ? QString(", pending %1/%2")
              .arg(m_measurementStateMachine.pendingPoints().size())
              .arg(m_measurementMode == measurement::MeasurementMode::Angle ? 4 : 2)
        : QString();
    m_statusLabel->setText(QString("Active view: %1\nMPR mode: %2\nMeasurement: %3%4 (%5)\nInstrument edit: %6\nDRR placement: %7\nPlan instruments: %8")
                               .arg(QString::fromUtf8(planeTitle(m_activeMprPlane)))
                               .arg(m_freeObliqueMode ? "Free oblique" : "Orthogonal")
                               .arg(QString::fromUtf8(measurementModeName(m_measurementMode)))
                               .arg(pendingMeasurement)
                               .arg(m_measurementStore.size())
                               .arg(editMode)
                               .arg(drrPlacement)
                               .arg(m_plan.instruments().size()));
}

void MprPlanVerificationWindow::refreshInstrumentList()
{
    const std::string selected = selectedInstrumentId();
    m_instrumentList->blockSignals(true);
    m_instrumentList->clear();
    int selectedRow = -1;
    int row = 0;
    for (const measurement::Instrument& instrument : m_plan.instruments()) {
        auto* item = new QListWidgetItem(instrumentText(instrument), m_instrumentList);
        item->setData(Qt::UserRole, QString::fromStdString(instrument.id));
        if (instrument.id == selected) {
            selectedRow = row;
        }
        ++row;
    }
    if (selectedRow >= 0) {
        m_instrumentList->setCurrentRow(selectedRow);
    }
    m_instrumentList->blockSignals(false);
    syncPlacementSelectionFromUi();
}

std::string MprPlanVerificationWindow::selectedInstrumentId() const
{
    const QListWidgetItem* item = m_instrumentList->currentItem();
    if (item == nullptr) {
        return {};
    }
    return item->data(Qt::UserRole).toString().toStdString();
}

void MprPlanVerificationWindow::syncSpinBoxesFromSelectedInstrument()
{
    const std::string id = selectedInstrumentId();
    const measurement::Instrument* instrument = id.empty() ? nullptr : m_plan.findInstrument(id);
    if (instrument == nullptr) {
        return;
    }

    m_syncingControls = true;
    m_label->setText(QString::fromStdString(instrument->label));
    m_length->setValue(instrument->lengthMm);
    m_diameter->setValue(instrument->diameterMm);
    m_syncingControls = false;
    updateInstrumentEditButton();
}

measurement::InstrumentPatch MprPlanVerificationWindow::patchFromControls() const
{
    measurement::InstrumentPatch patch;
    const std::string id = selectedInstrumentId();
    const measurement::Instrument* instrument = id.empty() ? nullptr : m_plan.findInstrument(id);
    if (instrument != nullptr) {
        patch.entryPointPatientMm = instrument->entryPointPatientMm;
        patch.directionPatientUnit = instrument->directionPatientUnit;
        patch.visible = instrument->visible;
        patch.locked = instrument->locked;
    }
    patch.lengthMm = m_length->value();
    patch.diameterMm = m_diameter->value();
    patch.label = m_label->text().toStdString();
    return patch;
}

void MprPlanVerificationWindow::addInstrument(measurement::InstrumentType type)
{
    const std::string id = (type == measurement::InstrumentType::GuidePin ? "pin-" : "screw-")
        + std::to_string(m_nextInstrumentIndex++);
    const measurement::Vec3d defaultDirection = activeCrosshairLineDirectionPatient();
    const double defaultLength = type == measurement::InstrumentType::GuidePin ? 70.0 : 45.0;
    const double defaultDiameter = type == measurement::InstrumentType::GuidePin ? 2.0 : 6.5;

    const auto result = type == measurement::InstrumentType::GuidePin
        ? m_placementController->createGuidePinAtCrosshair(id, m_mprState.crosshairPatientMm, defaultDirection, defaultLength, defaultDiameter, id)
        : m_placementController->createPedicleScrewAtCrosshair(id, m_mprState.crosshairPatientMm, defaultDirection, defaultLength, defaultDiameter, id);
    if (!result.ok()) {
        statusBar()->showMessage(QString::fromStdString(result.error().message), 6000);
        return;
    }
    m_placementController->setSelectedInstrumentId(id);

    refreshInstrumentList();
    for (int row = 0; row < m_instrumentList->count(); ++row) {
        if (m_instrumentList->item(row)->data(Qt::UserRole).toString().toStdString() == id) {
            m_instrumentList->setCurrentRow(row);
            break;
        }
    }
    syncSpinBoxesFromSelectedInstrument();
    refreshAll(true);
}

void MprPlanVerificationWindow::applyInstrumentPropertyEdits()
{
    if (m_syncingControls || !m_instrumentEditActive) {
        return;
    }
    const std::string id = selectedInstrumentId();
    if (id.empty() || id != m_editingInstrumentId) {
        return;
    }
    const auto result = m_planController->updateInstrument(id, patchFromControls());
    if (!result.ok()) {
        statusBar()->showMessage(QString::fromStdString(result.error().message + ": " + result.error().detail), 6000);
        syncSpinBoxesFromSelectedInstrument();
        return;
    }
    refreshAll(true);
}

void MprPlanVerificationWindow::removeSelectedInstrument()
{
    if (m_instrumentEditActive && !requestFinishInstrumentEdit()) {
        return;
    }
    const std::string id = selectedInstrumentId();
    if (id.empty()) {
        return;
    }
    const auto result = m_planController->removeInstrument(id);
    if (!result.ok()) {
        statusBar()->showMessage(QString::fromStdString(result.error().message), 6000);
        return;
    }
    refreshAll(true);
}

bool MprPlanVerificationWindow::jumpToInstrumentPlanningPose(const std::string& id)
{
    if (id.empty() || !m_volume.image) {
        return false;
    }

    const measurement::Instrument* instrument = m_plan.findInstrument(id);
    if (instrument == nullptr) {
        return false;
    }

    measurement::Vec3d direction = measurement::normalize(instrument->directionPatientUnit);
    if (!isFiniteVec(direction) || measurement::length(direction) <= 1.0e-6) {
        return false;
    }

    measurement::Vec3d voxel = measurement::patientToVoxel(m_volume.transform, instrument->entryPointPatientMm);
    if (!isFiniteVec(voxel)) {
        return false;
    }
    const measurement::Size3i dims = m_volume.metadata.dimensions;
    voxel.x = clampDouble(voxel.x, 0.0, static_cast<double>(dims.x - 1));
    voxel.y = clampDouble(voxel.y, 0.0, static_cast<double>(dims.y - 1));
    voxel.z = clampDouble(voxel.z, 0.0, static_cast<double>(dims.z - 1));
    m_mprState.crosshairPatientMm = measurement::voxelToPatient(m_volume.transform, voxel);
    if (!isFiniteVec(m_mprState.crosshairPatientMm)) {
        return false;
    }

    const std::array<measurement::MprPlane, 3> planes{
        measurement::MprPlane::Axial,
        measurement::MprPlane::Sagittal,
        measurement::MprPlane::Coronal,
    };

    measurement::MprPlane anchorPlane = measurement::MprPlane::Axial;
    measurement::Vec3d anchorNormal = {};
    double bestAnchorScore = -1.0;
    for (measurement::MprPlane plane : planes) {
        const measurement::Vec3d candidateNormal = measurement::planeNormalPatient(m_volume.metadata, plane);
        measurement::Vec3d projected = projectedOntoPlane(candidateNormal, direction);
        double projectedLength = measurement::length(projected);
        if (projectedLength > bestAnchorScore) {
            bestAnchorScore = projectedLength;
            anchorPlane = plane;
            anchorNormal = projected;
        }
    }
    if (measurement::length(anchorNormal) <= 1.0e-6) {
        anchorNormal = fallbackInPlaneAxis(direction);
    }
    anchorNormal = measurement::normalize(anchorNormal);
    if (!isFiniteVec(anchorNormal) || measurement::length(anchorNormal) <= 1.0e-6) {
        return false;
    }

    measurement::Vec3d lineNormal = measurement::normalize(measurement::cross(direction, anchorNormal));
    if (!isFiniteVec(lineNormal) || measurement::length(lineNormal) <= 1.0e-6) {
        lineNormal = fallbackInPlaneAxis(anchorNormal);
    }
    if (!isFiniteVec(lineNormal) || measurement::length(lineNormal) <= 1.0e-6) {
        return false;
    }

    measurement::MprPlane linePlane = measurement::MprPlane::Sagittal;
    double bestLineScore = -1.0;
    for (measurement::MprPlane plane : planes) {
        if (plane == anchorPlane) {
            continue;
        }
        const measurement::Vec3d candidateNormal = measurement::planeNormalPatient(m_volume.metadata, plane);
        const double score = std::abs(measurement::dot(candidateNormal, lineNormal));
        if (score > bestLineScore) {
            bestLineScore = score;
            linePlane = plane;
        }
    }
    const measurement::MprPlane otherPlane = remainingPlane(anchorPlane, linePlane);
    const measurement::Vec3d otherNormal = measurement::normalize(measurement::cross(anchorNormal, lineNormal));
    if (!isFiniteVec(otherNormal) || measurement::length(otherNormal) <= 1.0e-6) {
        return false;
    }

    m_activeMprPlane = anchorPlane;
    m_activeCrosshairLinePlane = linePlane;

    m_planeFrames[planeIndex(anchorPlane)] = frameWithNormal(m_planeFrames[planeIndex(anchorPlane)], anchorNormal);
    m_planeFrames[planeIndex(linePlane)] = frameWithNormal(m_planeFrames[planeIndex(linePlane)], lineNormal);
    m_planeFrames[planeIndex(otherPlane)] = frameWithNormal(m_planeFrames[planeIndex(otherPlane)], otherNormal);
    if (!m_freeObliqueMode) {
        orthogonalizePlaneFrames(anchorPlane, linePlane);
    }

    syncPlaneFrameOrigins();
    syncSlidersFromCrosshair();
    refreshAll(true);
    return true;
}

void MprPlanVerificationWindow::syncPlacementSelectionFromUi()
{
    m_placementController->setSelectedInstrumentId(selectedInstrumentId());
    refreshStatus();
}

void MprPlanVerificationWindow::toggleInstrumentEdit()
{
    if (m_instrumentEditActive) {
        (void)requestFinishInstrumentEdit();
        return;
    }

    const std::string id = selectedInstrumentId();
    if (id.empty()) {
        statusBar()->showMessage("Select an instrument before editing", 4000);
        return;
    }
    beginInstrumentEdit(id);
}

void MprPlanVerificationWindow::beginInstrumentEdit(const std::string& id)
{
    const measurement::Instrument* instrument = m_plan.findInstrument(id);
    if (instrument == nullptr) {
        statusBar()->showMessage("Selected instrument was not found", 4000);
        return;
    }
    if (instrument->locked) {
        statusBar()->showMessage("Locked instruments cannot be edited", 5000);
        return;
    }

    m_editingInstrumentId = id;
    m_editOriginalPatch.entryPointPatientMm = instrument->entryPointPatientMm;
    m_editOriginalPatch.directionPatientUnit = instrument->directionPatientUnit;
    m_editOriginalPatch.lengthMm = instrument->lengthMm;
    m_editOriginalPatch.diameterMm = instrument->diameterMm;
    m_editOriginalPatch.visible = instrument->visible;
    m_editOriginalPatch.locked = instrument->locked;
    m_editOriginalPatch.label = instrument->label;
    m_instrumentEditActive = true;
    m_instrumentList->setEnabled(false);
    m_placementController->setSelectedInstrumentId(id);
    m_placementController->setEditMode(measurement::InstrumentPlacementEditMode::BindEntryToCrosshair);

    const auto result = m_placementController->onCrosshairChanged(m_mprState.crosshairPatientMm);
    if (!result.ok()) {
        statusBar()->showMessage(QString::fromStdString(result.error().message), 6000);
        finishInstrumentEdit(false);
        return;
    }
    alignEditingInstrumentToCrosshairLine(m_activeMprPlane, m_activeCrosshairLinePlane);
    syncSpinBoxesFromSelectedInstrument();
    updateInstrumentEditButton();
    refreshAll(true);
}

bool MprPlanVerificationWindow::requestFinishInstrumentEdit()
{
    if (!m_instrumentEditActive) {
        return true;
    }

    const QMessageBox::StandardButton choice = QMessageBox::question(
        this,
        "Instrument edit",
        "Save changes to the selected instrument?",
        QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel,
        QMessageBox::Yes);
    if (choice == QMessageBox::Cancel) {
        return false;
    }

    finishInstrumentEdit(choice == QMessageBox::Yes);
    return true;
}

void MprPlanVerificationWindow::finishInstrumentEdit(bool saveChanges)
{
    const std::string id = m_editingInstrumentId;
    if (!saveChanges && !id.empty() && m_plan.findInstrument(id) != nullptr) {
        const auto restoreResult = m_planController->updateInstrument(id, m_editOriginalPatch);
        if (!restoreResult.ok()) {
            statusBar()->showMessage(QString::fromStdString(restoreResult.error().message), 6000);
        }
    }

    m_instrumentEditActive = false;
    m_editingInstrumentId.clear();
    m_placementController->setEditMode(measurement::InstrumentPlacementEditMode::None);
    syncPlacementSelectionFromUi();
    m_instrumentList->setEnabled(true);
    syncSpinBoxesFromSelectedInstrument();
    updateInstrumentEditButton();
    refreshAll(true);
}

void MprPlanVerificationWindow::updateInstrumentEditButton()
{
    if (m_editInstrumentButton == nullptr) {
        return;
    }
    m_editInstrumentButton->setText(m_instrumentEditActive ? "Finish edit" : "Edit selected");
}

void MprPlanVerificationWindow::activateMprPlane(measurement::MprPlane plane)
{
    m_activeMprPlane = plane;
    if (m_activeCrosshairLinePlane == m_activeMprPlane) {
        m_activeCrosshairLinePlane = m_activeMprPlane == measurement::MprPlane::Axial
            ? measurement::MprPlane::Sagittal
            : measurement::MprPlane::Axial;
    }
    refreshStatus();
}

void MprPlanVerificationWindow::alignEditingInstrumentToCrosshairLine(
    measurement::MprPlane viewPlane,
    measurement::MprPlane linePlane)
{
    if (!m_instrumentEditActive || m_editingInstrumentId.empty()) {
        return;
    }
    const measurement::Instrument* instrument = m_plan.findInstrument(m_editingInstrumentId);
    if (instrument == nullptr) {
        return;
    }

    measurement::Vec3d direction = crosshairLineDirectionPatient(viewPlane, linePlane);
    if (measurement::length(direction) <= 1.0e-6) {
        return;
    }
    if (measurement::dot(direction, instrument->directionPatientUnit) < 0.0) {
        direction = direction * -1.0;
    }

    measurement::InstrumentPatch patch;
    patch.entryPointPatientMm = instrument->entryPointPatientMm;
    patch.directionPatientUnit = direction;
    patch.lengthMm = instrument->lengthMm;
    patch.diameterMm = instrument->diameterMm;
    patch.visible = instrument->visible;
    patch.locked = instrument->locked;
    patch.label = instrument->label;

    const auto result = m_planController->updateInstrument(m_editingInstrumentId, patch);
    if (!result.ok()) {
        statusBar()->showMessage(QString::fromStdString(result.error().message), 6000);
    }
}

measurement::Vec3d MprPlanVerificationWindow::crosshairLineDirectionPatient(
    measurement::MprPlane viewPlane,
    measurement::MprPlane linePlane) const
{
    if (viewPlane == linePlane) {
        return {};
    }
    return stableCrosslineDirectionPatient(&m_volume, m_planeFrames, viewPlane, linePlane);
}

measurement::Vec3d MprPlanVerificationWindow::activeCrosshairLineDirectionPatient() const
{
    measurement::MprPlane linePlane = m_activeCrosshairLinePlane;
    if (linePlane == m_activeMprPlane) {
        linePlane = m_activeMprPlane == measurement::MprPlane::Axial
            ? measurement::MprPlane::Sagittal
            : measurement::MprPlane::Axial;
    }
    measurement::Vec3d direction = crosshairLineDirectionPatient(m_activeMprPlane, linePlane);
    if (measurement::length(direction) <= 1.0e-6) {
        direction = m_planeFrames[planeIndex(m_activeMprPlane)].horizontalPatientUnit;
    }
    return measurement::normalize(direction * -1.0);
}

measurement::ProjectManifest MprPlanVerificationWindow::makeManifest() const
{
    measurement::ProjectManifest manifest;
    manifest.dicomSourceFolder = m_volume.sourceFolder;
    manifest.studyUid = m_volume.studyUid;
    manifest.seriesUid = m_volume.seriesUid;
    manifest.dataHash = m_volume.dataHash;
    manifest.plan = m_plan;
    manifest.mprView.crosshairPatientMm = m_mprState.crosshairPatientMm;
    manifest.mprView.zoom = m_mprState.zoom;
    manifest.mprView.windowCenterHu = m_mprState.windowCenterHu;
    manifest.mprView.windowWidthHu = m_mprState.windowWidthHu;
    return manifest;
}

}  // namespace measurement_app
