#include "MprSliceWidget.h"

#include "measurement/core/MeasurementVisibility.h"

#include <QPainter>
#include <QPainterPath>
#include <QPen>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace measurement_app {
namespace {
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

}  // namespace

void MprSliceWidget::setMeasurements(const std::vector<measurement::MeasurementAnnotation>* measurements)
{
    m_measurements = measurements;
    update();
}

void MprSliceWidget::setMeasurementInteractionState(
    measurement::MeasurementMode mode,
    std::vector<measurement::Vec3d> pendingPoints,
    std::optional<measurement::Vec3d> hoverPoint,
    measurement::MeasurementId selectedId)
{
    m_measurementMode = mode;
    m_pendingMeasurementPoints = std::move(pendingPoints);
    m_measurementHoverPatientMm = hoverPoint;
    m_selectedMeasurementId = selectedId;
    update();
}

void MprSliceWidget::setMeasurementPointAddedCallback(
    std::function<void(measurement::MprPlane, measurement::Vec3d, measurement::MeasurementPlane)> callback)
{
    m_measurementPointAdded = std::move(callback);
}

void MprSliceWidget::setMeasurementHoverChangedCallback(
    std::function<void(measurement::MprPlane, std::optional<measurement::Vec3d>)> callback)
{
    m_measurementHoverChanged = std::move(callback);
}

void MprSliceWidget::setMeasurementCancelCallback(std::function<void()> callback)
{
    m_measurementCancelRequested = std::move(callback);
}

std::optional<measurement::MeasurementPlane> MprSliceWidget::currentMeasurementPlane() const
{
    const auto params = parameters();
    if (!params.ok()) {
        return std::nullopt;
    }

    const measurement::MprSliceFrame& frame = params.value().frame;
    const measurement::MprSliceRequest& request = params.value().request;
    return measurement::MeasurementPlane{
        frame.normalPatientUnit,
        frame.originPatientMm,
        std::max(request.pixelSpacingMm, 0.25),
    };
}

std::optional<measurement::Vec3d> MprSliceWidget::patientPointFromWidgetPosition(const QPoint& position) const
{
    if (m_image.isNull() || m_state == nullptr || m_volume == nullptr || !imageRect().contains(position)) {
        return std::nullopt;
    }

    return imagePointToPatient(widgetPointToImagePoint(position));
}

void MprSliceWidget::drawMeasurementOverlays(QPainter& painter)
{
    const auto slice = currentMeasurementPlane();
    if (!slice.has_value()) {
        return;
    }

    if (m_measurements != nullptr) {
        for (const measurement::MeasurementAnnotation& annotation : *m_measurements) {
            const measurement::MeasurementVisibilityResult result = measurement::measurement_visibility::evaluate(
                annotation,
                *slice,
                measurementViewTypeForPlane(m_plane));
            if (measurement::measurement_visibility::isMeasurementControlVisible(result)) {
                drawMeasurementAnnotation(painter, result);
            }
        }
    }

    drawMeasurementPreview(painter);
}

void MprSliceWidget::drawMeasurementAnnotation(QPainter& painter, const measurement::MeasurementVisibilityResult& result)
{
    const QColor color = result.selected ? QColor("#ffdd57") : QColor("#4cc9f0");
    const QSizeF bounds(static_cast<double>(m_image.width()), static_cast<double>(m_image.height()));
    const bool fullDisplay = result.level == measurement::MeasurementVisibilityLevel::FullDisplay;
    const std::vector<measurement::Vec3d>* points = &result.fullWorldPointsPatientMm;
    if (!fullDisplay) {
        points = &result.sectionWorldPointsPatientMm;
    }

    std::vector<QPointF> imagePoints;
    imagePoints.reserve(points->size());
    for (measurement::Vec3d point : *points) {
        imagePoints.push_back(patientToImagePoint(point));
    }

    if (result.type == measurement::MeasurementType::Distance && imagePoints.size() >= 2) {
        drawMeasurementPolyline(painter, imagePoints, color, fullDisplay ? 2.5 : 1.5);
        drawMeasurementHandles(painter, imagePoints, color);
        if (fullDisplay) {
            drawMeasurementLabels(
                painter,
                bounds,
                result.label,
                result.measurementText,
                result.displayText,
                imagePoints.back(),
                color);
        }
    } else if (result.type == measurement::MeasurementType::Angle && imagePoints.size() >= 4) {
        drawAngleMeasurement(painter, imagePoints, color, bounds);
        if (fullDisplay) {
            drawMeasurementLabels(
                painter,
                bounds,
                result.label,
                result.measurementText,
                result.displayText,
                angleLabelAnchor(imagePoints, bounds),
                color);
        }
    } else if (imagePoints.size() >= 2) {
        drawMeasurementPolyline(painter, imagePoints, color, fullDisplay ? 2.5 : 1.5);
        drawMeasurementHandles(painter, imagePoints, color);
    }
}

void MprSliceWidget::drawMeasurementPreview(QPainter& painter)
{
    if (m_measurementMode == measurement::MeasurementMode::Navigate || m_pendingMeasurementPoints.empty()) {
        return;
    }

    std::vector<measurement::Vec3d> previewPoints = m_pendingMeasurementPoints;
    if (m_measurementHoverPatientMm.has_value()) {
        previewPoints.push_back(*m_measurementHoverPatientMm);
    }
    if (previewPoints.empty()) {
        return;
    }

    std::vector<QPointF> imagePoints;
    imagePoints.reserve(previewPoints.size());
    for (measurement::Vec3d point : previewPoints) {
        imagePoints.push_back(patientToImagePoint(point));
    }

    if (m_measurementMode == measurement::MeasurementMode::Distance && imagePoints.size() != 2) {
        return;
    }
    if (m_measurementMode == measurement::MeasurementMode::Angle && imagePoints.size() > 4) {
        return;
    }

    QPen pen(Qt::white, 2.0);
    pen.setCosmetic(true);
    pen.setDashPattern({7.0, 5.0});
    painter.save();
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);
    if (m_measurementMode == measurement::MeasurementMode::Angle) {
        if (imagePoints.size() >= 2) {
            painter.drawLine(imagePoints[0], imagePoints[1]);
        }
        if (imagePoints.size() >= 4) {
            painter.drawLine(imagePoints[2], imagePoints[3]);
        } else if (imagePoints.size() == 3) {
            painter.drawLine(imagePoints[2], imagePoints[2]);
        }
    } else {
        painter.drawLine(imagePoints[0], imagePoints[1]);
    }
    painter.restore();

    const QColor accent("#4cc9f0");
    drawMeasurementHandles(painter, imagePoints, accent);

    QString previewLabel;
    if (m_measurementMode == measurement::MeasurementMode::Distance && previewPoints.size() == 2) {
        previewLabel = QString::fromStdString(
            measurement::MeasurementAnnotation::makeDistance(previewPoints[0], previewPoints[1]).measurementText());
    } else if (m_measurementMode == measurement::MeasurementMode::Angle) {
        previewLabel = "角度";
        if (previewPoints.size() == 4) {
            const auto annotation = measurement::MeasurementAnnotation::tryMakeAngle(
                previewPoints[0],
                previewPoints[1],
                previewPoints[2],
                previewPoints[3]);
            if (annotation.has_value()) {
                previewLabel = QString::fromStdString(annotation->measurementText());
            }
        }
    }
    if (!previewLabel.isEmpty() && !imagePoints.empty()) {
        const QSizeF bounds(static_cast<double>(m_image.width()), static_cast<double>(m_image.height()));
        drawMeasurementLabel(painter, bounds, previewLabel, imagePoints.back(), accent);
    }
}

}  // namespace measurement_app
