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

}  // namespace measurement_app