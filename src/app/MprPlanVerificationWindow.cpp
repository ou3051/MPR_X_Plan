#include "MprPlanVerificationWindow.h"

#include "DrrInteractionGeometry.h"
#include "InstrumentRenderModel.h"
#include "MprSliceWidget.h"
#include "PlanSceneWidget.h"
#include "XrayDisplayWidget.h"
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

MprPlanVerificationWindow::MprPlanVerificationWindow(QWidget* parent)
    : QMainWindow(parent)
{
    m_planController = std::make_unique<measurement::InstrumentPlanController>(m_plan);
    m_placementController = std::make_unique<measurement::InstrumentPlacementController>(m_plan);
    buildUi();
    loadStartupVolume();
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
