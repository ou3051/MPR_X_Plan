#include "MprPlanVerificationWindow.h"

#include "DrrInteractionGeometry.h"
#include "InstrumentRenderModel.h"
#include "measurement/dicom/DicomVolumeLoader.h"
#include "measurement/drr/CpuDrrEngine.h"
#include "measurement/drr/CudaDrrEngine.h"
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
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QProgressDialog>
#include <QPushButton>
#include <QSlider>
#include <QSplitter>
#include <QStatusBar>
#include <QStringList>
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
constexpr auto kDefaultDicomFolder = R"(D:\code\dicom)";
constexpr auto kFallbackValidatedDicomFolder = R"(D:\code\dicom_track_b1_contiguous_035_332)";

void appendUniqueCandidate(QStringList& candidates, const QString& folder)
{
    const QString trimmed = folder.trimmed();
    if (trimmed.isEmpty()) {
        return;
    }

    const QString normalized = QDir::toNativeSeparators(QDir::cleanPath(trimmed));
    if (!candidates.contains(normalized, Qt::CaseInsensitive)) {
        candidates.push_back(normalized);
    }
}

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

[[nodiscard]] int xrayPresetIndex(measurement::XrayPreset preset)
{
    return preset == measurement::XrayPreset::LAT ? 1 : 0;
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

[[nodiscard]] measurement::Vec3d reflectPointAcrossPlane(
    measurement::Vec3d point,
    measurement::Vec3d center,
    measurement::Vec3d planeNormal)
{
    return center + reflectAcrossNormal(point - center, planeNormal);
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
    ScopedModalBusyDialog(QWidget* parent, QString labelText)
        : m_dialog(new QProgressDialog(std::move(labelText), QString(), 0, 0, parent))
    {
        m_dialog->setWindowTitle("Updating Patient Position");
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
    if (m_volume != volume) {
        m_pan = {};
        m_zoom = 1.0;
    }
    m_volume = volume;
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
            measurement::makeErrorInfo("MPR_VIEW_NOT_READY", "MPR view is not ready."));
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
        m_renderStatus = params.ok() ? "MPR volume is unavailable." : QString::fromStdString(params.error().code + ": " + params.error().detail);
        update();
        return;
    }

    const auto reslice = m_resliceAdapter.reslice(*m_volume, stateForPlane(), m_request);
    if (!reslice.ok() || reslice.value().image == nullptr || !reslice.value().readyToRender) {
        m_instrumentSections.clear();
        m_image = QImage();
        m_renderStatus = reslice.ok()
            ? "MPR adapter returned no renderable image."
            : QString::fromStdString(reslice.error().code + ": " + reslice.error().detail);
        update();
        return;
    }

    m_image = imageFromVtkReslice(*reslice.value().image);
    if (m_image.isNull()) {
        m_renderStatus = "MPR adapter returned an empty vtkImageData.";
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

        const measurement::Vec3d directionPatient = measurement::normalize(
            measurement::cross(currentFrame.normalPatientUnit, (*m_linkedPlaneFrames)[index].normalPatientUnit));
        const QPointF direction = imageDirectionForPlane(currentFrame, (*m_linkedPlaneFrames)[index].normalPatientUnit);
        const double directionLength = std::hypot(direction.x(), direction.y());
        if (directionLength <= 1.0e-6) {
            continue;
        }

        lines[lineSlot].sourcePlane = static_cast<measurement::MprPlane>(index);
        lines[lineSlot].centerImage = center;
        lines[lineSlot].directionImage = {
            direction.x() / directionLength,
            direction.y() / directionLength,
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

void MprSliceWidget::drawCrossline(QPainter& painter, const CrosslineInfo& line, QColor color, bool drawHandle)
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
        painter.drawText(rectForImage, Qt::AlignCenter, m_renderStatus.isEmpty() ? "No renderable volume" : m_renderStatus);
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
        drawCrossline(painter, lines[0], QColor(255, 180, 0, 220), true);
        drawCrossline(painter, lines[1], QColor(0, 200, 255, 220), true);
        painter.setBrush(QColor(0, 255, 180, 220));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(crosshair, 4.0, 4.0);
        drawInstrumentOverlays(painter);
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

struct PlanSceneWidget::Impl {
    QVTKOpenGLNativeWidget* vtkWidget = nullptr;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkImageData> volumeImage;
    std::string lastVolumeSignature;
    bool cameraInitialized = false;
};

PlanSceneWidget::PlanSceneWidget(QWidget* parent)
    : QWidget(parent)
    , m_impl(std::make_unique<Impl>())
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    m_impl->renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    m_impl->renderer = vtkSmartPointer<vtkRenderer>::New();
    m_impl->renderer->SetBackground(0.08, 0.09, 0.11);
    m_impl->renderer->SetBackground2(0.16, 0.18, 0.22);
    m_impl->renderer->GradientBackgroundOn();
    m_impl->renderWindow->AddRenderer(m_impl->renderer);

    m_impl->vtkWidget = new QVTKOpenGLNativeWidget(m_impl->renderWindow, this);
    layout->addWidget(m_impl->vtkWidget);

    vtkNew<vtkInteractorStyleTrackballCamera> style;
    if (m_impl->vtkWidget->interactor() != nullptr) {
        m_impl->vtkWidget->interactor()->SetInteractorStyle(style);
    }
}

PlanSceneWidget::~PlanSceneWidget() = default;

void PlanSceneWidget::setVolume(const measurement::VolumeData* volume)
{
    m_volume = volume;
}

void PlanSceneWidget::setPlan(const measurement::SurgicalPlan* plan)
{
    m_plan = plan;
}

void PlanSceneWidget::setSelectedInstrumentId(std::string id)
{
    m_selectedInstrumentId = std::move(id);
}

void PlanSceneWidget::setDrrProjections(
    std::array<measurement::ProjectionParams, 2> projections,
    std::array<bool, 2> enabled)
{
    m_drrProjections = projections;
    m_drrProjectionEnabled = enabled;
}

void PlanSceneWidget::setDrrImages(std::array<QImage, 2> images)
{
    m_drrImages = std::move(images);
}

QSize PlanSceneWidget::minimumSizeHint() const
{
    return {320, 320};
}

std::string PlanSceneWidget::volumeSignature() const
{
    if (m_volume == nullptr) {
        return {};
    }
    return volumeGeometrySignature(*m_volume, false);
}

void PlanSceneWidget::resetCamera()
{
    if (m_impl->renderer != nullptr) {
        m_impl->renderer->ResetCamera();
        m_impl->cameraInitialized = true;
    }
    if (m_impl->renderWindow != nullptr) {
        m_impl->renderWindow->Render();
    }
}

namespace {

[[nodiscard]] std::array<double, 6> volumePatientBounds(const measurement::VolumeData& volume)
{
    const measurement::Size3i dimensions = volume.metadata.dimensions;
    std::array<double, 6> bounds{
        (std::numeric_limits<double>::max)(),
        (std::numeric_limits<double>::lowest)(),
        (std::numeric_limits<double>::max)(),
        (std::numeric_limits<double>::lowest)(),
        (std::numeric_limits<double>::max)(),
        (std::numeric_limits<double>::lowest)(),
    };
    const std::array<double, 2> xs{0.0, static_cast<double>(std::max(dimensions.x - 1, 0))};
    const std::array<double, 2> ys{0.0, static_cast<double>(std::max(dimensions.y - 1, 0))};
    const std::array<double, 2> zs{0.0, static_cast<double>(std::max(dimensions.z - 1, 0))};
    for (double x : xs) {
        for (double y : ys) {
            for (double z : zs) {
                const measurement::Vec3d patient = measurement::voxelToPatient(volume.transform, {x, y, z});
                bounds[0] = std::min(bounds[0], patient.x);
                bounds[1] = std::max(bounds[1], patient.x);
                bounds[2] = std::min(bounds[2], patient.y);
                bounds[3] = std::max(bounds[3], patient.y);
                bounds[4] = std::min(bounds[4], patient.z);
                bounds[5] = std::max(bounds[5], patient.z);
            }
        }
    }
    return bounds;
}

[[nodiscard]] vtkSmartPointer<vtkImageData> makePatientVolumeImageData(const measurement::VolumeData& volume)
{
    if (!volume.image) {
        return nullptr;
    }

    const measurement::Size3i dimensions = volume.metadata.dimensions;
    if (dimensions.x <= 0 || dimensions.y <= 0 || dimensions.z <= 0) {
        return nullptr;
    }

    const measurement::Vec3d row = measurement::normalize(volume.metadata.rowDirectionPatient);
    const measurement::Vec3d column = measurement::normalize(volume.metadata.columnDirectionPatient);
    const measurement::Vec3d slice = measurement::length(volume.metadata.sliceDirectionPatient) > 0.0
        ? measurement::normalize(volume.metadata.sliceDirectionPatient)
        : measurement::normalize(measurement::cross(row, column));

    vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New();
    image->SetDimensions(dimensions.x, dimensions.y, dimensions.z);
    image->SetSpacing(
        volume.metadata.spacingMm.x,
        volume.metadata.spacingMm.y,
        volume.metadata.spacingMm.z);
    image->SetOrigin(
        volume.metadata.originPatientMm.x,
        volume.metadata.originPatientMm.y,
        volume.metadata.originPatientMm.z);
    image->SetDirectionMatrix(
        row.x, column.x, slice.x,
        row.y, column.y, slice.y,
        row.z, column.z, slice.z);
    image->AllocateScalars(VTK_SHORT, 1);

    for (int k = 0; k < dimensions.z; ++k) {
        for (int j = 0; j < dimensions.y; ++j) {
            for (int i = 0; i < dimensions.x; ++i) {
                auto* voxel = static_cast<int16_t*>(image->GetScalarPointer(i, j, k));
                *voxel = volume.image->voxelHu(i, j, k);
            }
        }
    }
    return image;
}

[[nodiscard]] vtkSmartPointer<vtkVolume> makeMedicalVolumeActor(vtkImageData* image)
{
    if (image == nullptr) {
        return nullptr;
    }

    vtkNew<vtkSmartVolumeMapper> mapper;
    mapper->SetInputData(image);
    mapper->SetBlendModeToComposite();
    mapper->SetRequestedRenderModeToGPU();

    vtkNew<vtkColorTransferFunction> color;
    color->AddRGBPoint(-1000.0, 0.0, 0.0, 0.0);
    color->AddRGBPoint(-150.0, 0.18, 0.18, 0.2);
    color->AddRGBPoint(180.0, 0.58, 0.55, 0.5);
    color->AddRGBPoint(700.0, 0.86, 0.82, 0.72);
    color->AddRGBPoint(1500.0, 1.0, 0.96, 0.86);

    vtkNew<vtkPiecewiseFunction> opacity;
    opacity->AddPoint(-1000.0, 0.0);
    opacity->AddPoint(-250.0, 0.0);
    opacity->AddPoint(100.0, 0.012);
    opacity->AddPoint(350.0, 0.035);
    opacity->AddPoint(700.0, 0.09);
    opacity->AddPoint(1500.0, 0.18);

    vtkNew<vtkVolumeProperty> property;
    property->SetColor(color);
    property->SetScalarOpacity(opacity);
    property->SetInterpolationTypeToLinear();
    property->ShadeOn();
    property->SetAmbient(0.25);
    property->SetDiffuse(0.72);
    property->SetSpecular(0.18);
    property->SetSpecularPower(12.0);

    vtkSmartPointer<vtkVolume> actor = vtkSmartPointer<vtkVolume>::New();
    actor->SetMapper(mapper);
    actor->SetProperty(property);
    return actor;
}

[[nodiscard]] vtkSmartPointer<vtkActor> makeInstrumentActor(
    const InstrumentRenderMeshSegment& renderMesh)
{
    vtkNew<vtkPoints> points;
    points->SetNumberOfPoints(static_cast<vtkIdType>(renderMesh.mesh.vertices.size()));
    for (size_t index = 0; index < renderMesh.mesh.vertices.size(); ++index) {
        const measurement::Vec3d vertex = renderMesh.mesh.vertices[index];
        points->SetPoint(static_cast<vtkIdType>(index), vertex.x, vertex.y, vertex.z);
    }

    vtkNew<vtkCellArray> polys;
    for (size_t index = 0; index + 2 < renderMesh.mesh.indices.size(); index += 3) {
        const vtkIdType triangle[3]{
            static_cast<vtkIdType>(renderMesh.mesh.indices[index]),
            static_cast<vtkIdType>(renderMesh.mesh.indices[index + 1]),
            static_cast<vtkIdType>(renderMesh.mesh.indices[index + 2]),
        };
        polys->InsertNextCell(3, triangle);
    }

    vtkNew<vtkPolyData> polyData;
    polyData->SetPoints(points);
    polyData->SetPolys(polys);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(polyData);

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    const std::array<double, 3> color = renderMesh.segment.style.color;
    actor->GetProperty()->SetColor(color[0], color[1], color[2]);
    actor->GetProperty()->SetOpacity(renderMesh.segment.style.opacity);
    actor->GetProperty()->SetSpecular(0.35);
    actor->GetProperty()->SetSpecularPower(24.0);
    return actor;
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

[[nodiscard]] vtkSmartPointer<vtkActor> makeDrrSourceActor(
    const measurement::ProjectionParams& projection,
    const std::array<double, 3>& color)
{
    vtkNew<vtkSphereSource> sphere;
    // Scale the marker with SID so it remains visible for both compact synthetic
    // phantoms and full-size CT volumes without becoming the dominant object.
    sphere->SetRadius(std::clamp(projection.sidMm * 0.012, 4.0, 14.0));
    sphere->SetThetaResolution(24);
    sphere->SetPhiResolution(24);
    sphere->SetCenter(
        projection.sourcePosPatientMm.x,
        projection.sourcePosPatientMm.y,
        projection.sourcePosPatientMm.z);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(sphere->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(color[0], color[1], color[2]);
    actor->GetProperty()->SetSpecular(0.4);
    actor->GetProperty()->SetSpecularPower(18.0);
    return actor;
}

[[nodiscard]] vtkSmartPointer<vtkActor> makeDrrDetectorActor(
    const measurement::ProjectionParams& projection,
    const std::array<double, 3>& color)
{
    const double halfWidthMm = 0.5
        * static_cast<double>(std::max(1, projection.detectorWidth))
        * projection.pixelSpacingMm;
    const double halfHeightMm = 0.5
        * static_cast<double>(std::max(1, projection.detectorHeight))
        * projection.pixelSpacingMm;
    const measurement::Vec3d u = measurement::normalize(projection.detectorUPatientUnit);
    const measurement::Vec3d v = measurement::normalize(projection.detectorVPatientUnit);
    const measurement::Vec3d c = projection.detectorCenterPatientMm;

    const measurement::Vec3d p0 = c - u * halfWidthMm - v * halfHeightMm;
    const measurement::Vec3d p1 = c + u * halfWidthMm - v * halfHeightMm;
    const measurement::Vec3d p2 = c - u * halfWidthMm + v * halfHeightMm;

    vtkNew<vtkPlaneSource> plane;
    plane->SetOrigin(p0.x, p0.y, p0.z);
    plane->SetPoint1(p1.x, p1.y, p1.z);
    plane->SetPoint2(p2.x, p2.y, p2.z);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(plane->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(color[0], color[1], color[2]);
    actor->GetProperty()->SetRepresentationToWireframe();
    actor->GetProperty()->SetLineWidth(2.0);
    actor->GetProperty()->SetOpacity(0.82);
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

[[nodiscard]] vtkSmartPointer<vtkActor> makeDrrTexturedDetectorActor(
    const measurement::ProjectionParams& projection,
    const QImage& image,
    double opacity)
{
    vtkSmartPointer<vtkImageData> vtkImage = makeVtkRgbaImage(image);
    if (vtkImage == nullptr) {
        return nullptr;
    }

    const double halfWidthMm = 0.5
        * static_cast<double>(std::max(1, projection.detectorWidth))
        * projection.pixelSpacingMm;
    const double halfHeightMm = 0.5
        * static_cast<double>(std::max(1, projection.detectorHeight))
        * projection.pixelSpacingMm;
    const measurement::Vec3d u = measurement::normalize(projection.detectorUPatientUnit);
    const measurement::Vec3d v = measurement::normalize(projection.detectorVPatientUnit);
    const measurement::Vec3d c = projection.detectorCenterPatientMm;

    const measurement::Vec3d p0 = c - u * halfWidthMm - v * halfHeightMm;
    const measurement::Vec3d p1 = c + u * halfWidthMm - v * halfHeightMm;
    const measurement::Vec3d p2 = c - u * halfWidthMm + v * halfHeightMm;

    vtkNew<vtkPlaneSource> plane;
    plane->SetOrigin(p0.x, p0.y, p0.z);
    plane->SetPoint1(p1.x, p1.y, p1.z);
    plane->SetPoint2(p2.x, p2.y, p2.z);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(plane->GetOutputPort());

    vtkNew<vtkTexture> texture;
    texture->SetInputData(vtkImage);
    texture->InterpolateOn();

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->SetTexture(texture);
    actor->GetProperty()->SetColor(1.0, 1.0, 1.0);
    actor->GetProperty()->SetOpacity(opacity);
    actor->GetProperty()->LightingOff();
    return actor;
}

void addDrrProjectionActors(
    vtkRenderer& renderer,
    const measurement::ProjectionParams& projection,
    const QImage& image,
    bool lateral)
{
    const std::array<double, 3> sourceColor = lateral
        ? std::array<double, 3>{1.0, 0.56, 0.20}
        : std::array<double, 3>{1.0, 0.86, 0.18};
    const std::array<double, 3> detectorColor = lateral
        ? std::array<double, 3>{0.66, 0.58, 1.0}
        : std::array<double, 3>{0.20, 0.72, 1.0};
    const std::array<double, 3> rayColor = lateral
        ? std::array<double, 3>{1.0, 0.44, 0.30}
        : std::array<double, 3>{0.20, 1.0, 0.64};

    if (!image.isNull()) {
        vtkSmartPointer<vtkActor> texturedDetector = makeDrrTexturedDetectorActor(projection, image, 0.72);
        if (texturedDetector != nullptr) {
            renderer.AddActor(texturedDetector);
        }
    }
    renderer.AddActor(makeDrrSourceActor(projection, sourceColor));
    renderer.AddActor(makeDrrDetectorActor(projection, detectorColor));

    const double halfWidthMm = 0.5
        * static_cast<double>(std::max(1, projection.detectorWidth))
        * projection.pixelSpacingMm;
    const double halfHeightMm = 0.5
        * static_cast<double>(std::max(1, projection.detectorHeight))
        * projection.pixelSpacingMm;
    const measurement::Vec3d u = measurement::normalize(projection.detectorUPatientUnit);
    const measurement::Vec3d v = measurement::normalize(projection.detectorVPatientUnit);

    const auto detectorPoint = [&](double su, double sv) {
        return projection.detectorCenterPatientMm + u * (su * halfWidthMm) + v * (sv * halfHeightMm);
    };

    // Match the reference xray viewport: center ray plus a few detector edge samples
    // make the simulated C-arm pose readable without drawing every detector pixel ray.
    const std::array<std::pair<double, double>, 7> samples{{
        {-1.0, 0.0},
        {-0.5, 0.0},
        {0.0, 0.0},
        {0.5, 0.0},
        {1.0, 0.0},
        {0.0, -1.0},
        {0.0, 1.0},
    }};
    for (size_t index = 0; index < samples.size(); ++index) {
        const measurement::Vec3d target = detectorPoint(samples[index].first, samples[index].second);
        renderer.AddActor(makeLineActor(
            projection.sourcePosPatientMm,
            target,
            rayColor,
            index == 2 ? 2.6 : 1.2,
            index == 2 ? 0.9 : 0.46));
    }
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

}  // namespace

void PlanSceneWidget::refreshScene()
{
    rebuildScene();
}

void PlanSceneWidget::rebuildScene()
{
    if (m_impl->renderer == nullptr || m_impl->renderWindow == nullptr) {
        return;
    }

    const std::string signature = volumeSignature();
    const bool volumeChanged = signature != m_impl->lastVolumeSignature;
    if (volumeChanged) {
        m_impl->lastVolumeSignature = signature;
        m_impl->volumeImage = m_volume != nullptr && m_volume->image
            ? makePatientVolumeImageData(*m_volume)
            : nullptr;
        m_impl->cameraInitialized = false;
    }

    m_impl->renderer->RemoveAllViewProps();

    std::array<double, 6> bounds{-80.0, 80.0, -80.0, 80.0, -80.0, 80.0};
    if (m_volume != nullptr && m_volume->image) {
        bounds = volumePatientBounds(*m_volume);
        vtkNew<vtkCubeSource> source;
        source->SetBounds(bounds.data());

        vtkNew<vtkPolyDataMapper> mapper;
        mapper->SetInputConnection(source->GetOutputPort());

        vtkNew<vtkActor> outline;
        outline->SetMapper(mapper);
        outline->GetProperty()->SetRepresentationToWireframe();
        outline->GetProperty()->SetColor(0.58, 0.64, 0.72);
        outline->GetProperty()->SetOpacity(0.45);
        outline->GetProperty()->SetLineWidth(1.0);
        m_impl->renderer->AddActor(outline);
    }

    if (m_impl->volumeImage != nullptr) {
        vtkSmartPointer<vtkVolume> medicalVolume = makeMedicalVolumeActor(m_impl->volumeImage);
        if (medicalVolume != nullptr) {
            m_impl->renderer->AddVolume(medicalVolume);
        }
    }

    if (m_plan != nullptr) {
        InstrumentRenderModelBuilder builder;
        const std::vector<InstrumentRenderMeshSegment> meshes =
            builder.buildVisibleMeshSegments(*m_plan, m_selectedInstrumentId, 32);
        for (const InstrumentRenderMeshSegment& mesh : meshes) {
            m_impl->renderer->AddActor(makeInstrumentActor(mesh));
        }
    }

    for (size_t index = 0; index < m_drrProjections.size(); ++index) {
        if (m_drrProjectionEnabled[index]) {
            addDrrProjectionActors(*m_impl->renderer, m_drrProjections[index], m_drrImages[index], index == 1);
        }
    }

    if (!m_impl->cameraInitialized) {
        m_impl->renderer->ResetCamera();
        vtkCamera* camera = m_impl->renderer->GetActiveCamera();
        if (camera != nullptr) {
            camera->Azimuth(35.0);
            camera->Elevation(22.0);
            camera->Dolly(1.2);
            m_impl->renderer->ResetCameraClippingRange();
        }
        m_impl->cameraInitialized = true;
    }

    m_impl->renderWindow->Render();
}

class XrayDisplayWidget final : public QWidget {
public:
    XrayDisplayWidget(QString title, measurement::XrayPreset preset, QWidget* parent = nullptr)
        : QWidget(parent)
        , m_title(std::move(title))
        , m_preset(preset)
    {
        setMinimumSize(260, 220);
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(3);

        m_renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
        m_renderer = vtkSmartPointer<vtkRenderer>::New();
        m_renderer->SetBackground(0.07, 0.08, 0.10);
        m_renderer->SetBackground2(0.13, 0.15, 0.18);
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
        m_captionLabel->setMinimumHeight(22);
        m_captionLabel->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        m_captionLabel->setStyleSheet("QLabel { color: #cfd5df; padding-left: 6px; }");
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

MprPlanVerificationWindow::MprPlanVerificationWindow(QString startupDicomFolder, QWidget* parent)
    : QMainWindow(parent)
{
    m_planController = std::make_unique<measurement::InstrumentPlanController>(m_plan);
    m_placementController = std::make_unique<measurement::InstrumentPlacementController>(m_plan);
    buildUi();
    loadStartupVolume(std::move(startupDicomFolder));
}

void MprPlanVerificationWindow::buildUi()
{
    setWindowTitle("Interactive MPR Test");

    auto* central = new QWidget(this);
    auto* root = new QHBoxLayout(central);
    auto* splitter = new QSplitter(Qt::Horizontal, central);
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
    auto* xrayLayout = new QVBoxLayout(xrayPanel);
    xrayLayout->setContentsMargins(6, 6, 6, 6);
    m_apXrayView = new XrayDisplayWidget("AP X-ray", measurement::XrayPreset::AP, xrayPanel);
    m_latXrayView = new XrayDisplayWidget("LAT X-ray", measurement::XrayPreset::LAT, xrayPanel);
    xrayLayout->addWidget(m_apXrayView, 1);
    xrayLayout->addWidget(m_latXrayView, 1);

    auto* controlPanel = new QWidget(splitter);
    auto* controls = new QVBoxLayout(controlPanel);

    auto* loadGrid = new QGridLayout();
    auto* syntheticButton = new QPushButton("Synthetic phantom", controlPanel);
    auto* dicomButton = new QPushButton("Load DICOM", controlPanel);
    auto* resetViewsButton = new QPushButton("Reset Views", controlPanel);
    auto* saveButton = new QPushButton("Save .mprproj", controlPanel);
    m_freeObliqueButton = new QPushButton("Free oblique: Off", controlPanel);
    m_freeObliqueButton->setCheckable(true);
    // Keep the command buttons compact so the right-side control columns can stay
    // near one third of the window without forcing the image panels to shrink.
    loadGrid->addWidget(syntheticButton, 0, 0);
    loadGrid->addWidget(dicomButton, 0, 1);
    loadGrid->addWidget(resetViewsButton, 1, 0);
    loadGrid->addWidget(saveButton, 1, 1);
    loadGrid->addWidget(m_freeObliqueButton, 2, 0, 1, 2);
    controls->addLayout(loadGrid);

    m_volumeLabel = new QLabel(controlPanel);
    m_volumeLabel->setWordWrap(true);
    controls->addWidget(m_volumeLabel);

    auto* postureGroup = new QGroupBox("Patient position", controlPanel);
    auto* postureLayout = new QFormLayout(postureGroup);
    m_patientPostureCombo = new QComboBox(postureGroup);
    m_patientPostureCombo->addItem("Supine", false);
    m_patientPostureCombo->addItem("Prone", true);
    m_headFeetDirectionCombo = new QComboBox(postureGroup);
    m_headFeetDirectionCombo->addItem("Head first", false);
    m_headFeetDirectionCombo->addItem("Feet first", true);
    postureLayout->addRow("Body posture", m_patientPostureCombo);
    postureLayout->addRow("Entry direction", m_headFeetDirectionCombo);
    controls->addWidget(postureGroup);

    auto* crosshairGroup = new QGroupBox("Crosshair voxel", controlPanel);
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
    controls->addWidget(crosshairGroup);

    auto* instrumentGroup = new QGroupBox("Plan instruments", controlPanel);
    auto* instrumentLayout = new QVBoxLayout(instrumentGroup);
    m_instrumentList = new QListWidget(instrumentGroup);
    instrumentLayout->addWidget(m_instrumentList);

    auto* form = new QFormLayout();
    m_label = new QLineEdit(instrumentGroup);
    m_length = makeSpin(1.0, 300.0, 55.0, 1.0);
    m_diameter = makeSpin(0.5, 20.0, 2.0, 0.5);
    form->addRow("Name", m_label);
    form->addRow("Length mm", m_length);
    form->addRow("Diameter mm", m_diameter);
    instrumentLayout->addLayout(form);

    auto* instrumentButtons = new QGridLayout();
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
    m_drrPinButton = new QPushButton("DRR Pin", instrumentGroup);
    m_drrScrewButton = new QPushButton("DRR Screw", instrumentGroup);
    m_drrCancelButton = new QPushButton("Cancel DRR placement", instrumentGroup);
    m_drrPinButton->setCheckable(true);
    m_drrScrewButton->setCheckable(true);
    drrPlacementButtons->addWidget(m_drrPinButton, 0, 0);
    drrPlacementButtons->addWidget(m_drrScrewButton, 0, 1);
    drrPlacementButtons->addWidget(m_drrCancelButton, 1, 0, 1, 2);
    instrumentLayout->addLayout(drrPlacementButtons);
    controls->addWidget(instrumentGroup, 1);

    m_statusLabel = new QLabel(controlPanel);
    m_statusLabel->setWordWrap(true);
    controls->addWidget(m_statusLabel);
    controlPanel->setMinimumWidth(300);

    auto* drrPanel = new QWidget(splitter);
    auto* drrPanelLayout = new QVBoxLayout(drrPanel);
    auto* drrGroup = new QGroupBox("DRR 参数", drrPanel);
    auto* drrGroupLayout = new QVBoxLayout(drrGroup);
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
    drrPanel->setMinimumWidth(220);

    splitter->addWidget(mprPanel);
    splitter->addWidget(xrayPanel);
    splitter->addWidget(controlPanel);
    splitter->addWidget(drrPanel);
    splitter->setChildrenCollapsible(false);
    // Initial layout target: MPR+3D and DRR image columns occupy about two thirds
    // of the window, leaving the planning and DRR parameter columns the remaining
    // third. Stretch factors keep that relationship when the user resizes.
    splitter->setStretchFactor(0, 4);
    splitter->setStretchFactor(1, 2);
    splitter->setStretchFactor(2, 2);
    splitter->setStretchFactor(3, 1);
    splitter->setSizes({690, 350, 300, 220});

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

    connect(syntheticButton, &QPushButton::clicked, this, [this]() { loadSyntheticVolume(); });
    connect(dicomButton, &QPushButton::clicked, this, [this]() { loadDicomFolder(); });
    connect(resetViewsButton, &QPushButton::clicked, this, [this]() { resetAllViews(); });
    connect(saveButton, &QPushButton::clicked, this, [this]() { saveProject(); });
    connect(m_freeObliqueButton, &QPushButton::toggled, this, [this](bool checked) { setFreeObliqueMode(checked); });
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
    const auto instrumentSelectionChanged = [this](bool forceJump) {
        syncPlacementSelectionFromUi();
        const bool jumped = forceJump
            ? jumpToInstrumentPlanningPose(selectedInstrumentId())
            : jumpToInstrumentPlanningPose(selectedInstrumentId());
        syncSpinBoxesFromSelectedInstrument();
        if (!jumped) {
            refreshAll(true);
        }
    };
    connect(m_instrumentList, &QListWidget::currentRowChanged, this, [instrumentSelectionChanged]() {
        instrumentSelectionChanged(false);
    });
    connect(m_instrumentList, &QListWidget::itemClicked, this, [instrumentSelectionChanged](QListWidgetItem*) {
        instrumentSelectionChanged(true);
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

    resize(1560, 840);
}

void MprPlanVerificationWindow::loadSyntheticVolume()
{
    m_volume = makeSyntheticVolume();
    resetPatientPositionControls();
    initializePlaneFrames();
    resetCrosshairToVolumeCenter();
    refreshAll(true);
}

void MprPlanVerificationWindow::loadStartupVolume(const QString& startupDicomFolder)
{
    QStringList candidates;
    appendUniqueCandidate(candidates, startupDicomFolder);
    appendUniqueCandidate(candidates, qEnvironmentVariable("MEASUREMENT_MPR_DICOM_FOLDER"));
    appendUniqueCandidate(candidates, QString::fromUtf8(kDefaultDicomFolder));
    appendUniqueCandidate(candidates, QString::fromUtf8(kFallbackValidatedDicomFolder));

    QStringList failures;
    for (const QString& candidate : candidates) {
        QString failureMessage;
        if (tryLoadDicomFolder(candidate, &failureMessage)) {
            QString startupMessage = QString("Loaded startup DICOM: %1").arg(candidate);
            if (!failures.isEmpty()) {
                startupMessage += QString(" (after fallback; previous failure: %1)").arg(failures.constLast());
            }
            statusBar()->showMessage(startupMessage, 15000);
            return;
        }
        failures.push_back(failureMessage);
    }

    loadSyntheticVolume();
    const QString failureSuffix = failures.isEmpty()
        ? QString("No startup DICOM folder was configured.")
        : QString("Last startup DICOM failure: %1").arg(failures.constLast());
    statusBar()->showMessage(
        QString("Loaded synthetic phantom. %1").arg(failureSuffix),
        15000);
}

void MprPlanVerificationWindow::loadDicomFolder()
{
    const QString initialDirectory = QFileInfo(QString::fromStdString(m_volume.sourceFolder)).isDir()
        ? QString::fromStdString(m_volume.sourceFolder)
        : QString::fromUtf8(kDefaultDicomFolder);
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
        return;
    }

    statusBar()->showMessage(QString("Loaded DICOM: %1").arg(QDir::toNativeSeparators(folder)), 6000);
}

bool MprPlanVerificationWindow::tryLoadDicomFolder(const QString& folder, QString* failureMessage)
{
    measurement::DicomVolumeLoader loader;
    const auto loaded = loader.loadFolder(folder.toStdString());
    if (!loaded.ok()) {
        if (failureMessage != nullptr) {
            *failureMessage = summarizeLoadFailure(QDir::toNativeSeparators(folder), loaded.error());
        }
        return false;
    }

    m_volume = loaded.value();
    resetPatientPositionControls();
    initializePlaneFrames();
    resetCrosshairToVolumeCenter();
    refreshAll(true);
    return true;
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

void MprPlanVerificationWindow::resetCrosshairToVolumeCenter()
{
    const measurement::Size3i dims = m_volume.metadata.dimensions;
    setCrosshairVoxel({
        static_cast<double>(dims.x - 1) * 0.5,
        static_cast<double>(dims.y - 1) * 0.5,
        static_cast<double>(dims.z - 1) * 0.5,
    });
}

void MprPlanVerificationWindow::setCrosshairVoxel(measurement::Vec3d voxel)
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

    ScopedModalBusyDialog busyDialog(this, "Updating patient position...");

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

    const measurement::Vec3d voxel = measurement::patientToVoxel(m_volume.transform, m_mprState.crosshairPatientMm);
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
    m_statusLabel->setText(QString("Crosshair patient %1\nCrosshair voxel %2\nActive view: %3\nMPR mode: %4\nInstrument edit: %5\nDRR placement: %6\nPlan instruments: %7")
                               .arg(vecText(m_mprState.crosshairPatientMm))
                               .arg(vecText(voxel))
                               .arg(QString::fromUtf8(planeTitle(m_activeMprPlane)))
                               .arg(m_freeObliqueMode ? "Free oblique" : "Orthogonal")
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

void MprPlanVerificationWindow::applyInstrumentEdits()
{
    const std::string id = selectedInstrumentId();
    if (id.empty()) {
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

void MprPlanVerificationWindow::setSelectedVisible(bool visible)
{
    if (m_syncingControls) {
        return;
    }
    const std::string id = selectedInstrumentId();
    if (id.empty()) {
        return;
    }
    const auto result = m_planController->setInstrumentVisible(id, visible);
    if (!result.ok()) {
        statusBar()->showMessage(QString::fromStdString(result.error().message), 6000);
    }
    refreshAll(true);
}

void MprPlanVerificationWindow::setSelectedLocked(bool locked)
{
    if (m_syncingControls) {
        return;
    }
    const std::string id = selectedInstrumentId();
    if (id.empty()) {
        return;
    }
    measurement::InstrumentPatch patch = patchFromControls();
    patch.locked = locked;
    const auto result = m_planController->updateInstrument(id, patch);
    if (!result.ok()) {
        statusBar()->showMessage(QString::fromStdString(result.error().message), 6000);
        syncSpinBoxesFromSelectedInstrument();
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
    return measurement::normalize(measurement::cross(
        m_planeFrames[planeIndex(viewPlane)].normalPatientUnit,
        m_planeFrames[planeIndex(linePlane)].normalPatientUnit));
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
    return measurement::normalize(direction);
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
