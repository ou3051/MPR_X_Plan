#include "MprPlanVerificationWindow.h"

#include <QVTKOpenGLNativeWidget.h>
#include <QVBoxLayout>

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
#include <limits>
#include <utility>

namespace measurement_app {
namespace {

[[nodiscard]] bool isFiniteVec(measurement::Vec3d value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
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

}  // namespace

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
}  // namespace measurement_app
