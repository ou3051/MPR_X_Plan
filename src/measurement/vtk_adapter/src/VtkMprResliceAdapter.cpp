#include "measurement/vtk/VtkMprResliceAdapter.h"

#include <string>
#include <utility>

#if MEASUREMENT_HAVE_VTK
#include <vtkImageData.h>
#include <vtkImageReslice.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkSmartPointer.h>

#include <cstdint>
#endif

namespace measurement {

namespace {

[[nodiscard]] ErrorInfo mprError(const char* code, const char* message, std::string detail = {})
{
    return makeErrorInfo(code, message, std::move(detail), true);
}

[[nodiscard]] bool sameDimensions(Size3i lhs, Size3i rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

#if MEASUREMENT_HAVE_VTK
[[nodiscard]] Result<vtkSmartPointer<vtkImageData>> makeVtkImageData(const VolumeData& volume)
{
    if (!volume.image) {
        return Result<vtkSmartPointer<vtkImageData>>::failure(mprError(
            "MPR_VOLUME_EMPTY",
            "MPR reslice requires a volume image."));
    }

    const Size3i dimensions = volume.metadata.dimensions;
    if (!sameDimensions(volume.image->dimensions(), dimensions)) {
        return Result<vtkSmartPointer<vtkImageData>>::failure(mprError(
            "MPR_VOLUME_METADATA_INVALID",
            "MPR volume image dimensions do not match metadata.",
            "image dimensions and metadata dimensions must match."));
    }

    vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New();
    image->SetDimensions(dimensions.x, dimensions.y, dimensions.z);
    image->SetSpacing(1.0, 1.0, 1.0);
    image->SetOrigin(0.0, 0.0, 0.0);
    image->AllocateScalars(VTK_SHORT, 1);

    for (int k = 0; k < dimensions.z; ++k) {
        for (int j = 0; j < dimensions.y; ++j) {
            for (int i = 0; i < dimensions.x; ++i) {
                auto* voxel = static_cast<int16_t*>(image->GetScalarPointer(i, j, k));
                *voxel = volume.image->voxelHu(i, j, k);
            }
        }
    }

    return Result<vtkSmartPointer<vtkImageData>>::success(image);
}

[[nodiscard]] Vec3d patientVectorToVoxelVector(const VolumeTransform& transform, Vec3d vector)
{
    return transformVector(transform.patientToVoxel, vector);
}
#endif

}  // namespace

struct VtkMprResliceAdapter::Impl {
#if MEASUREMENT_HAVE_VTK
    vtkSmartPointer<vtkImageData> sourceVolume;
    vtkSmartPointer<vtkImageData> lastImage;
    Size3i lastVolumeDimensions{};
    std::string lastSeriesUid;
    std::string lastStudyUid;
    std::string lastDataHash;
#endif
};

VtkMprResliceAdapter::VtkMprResliceAdapter()
    : m_impl(std::make_unique<Impl>())
{
}

VtkMprResliceAdapter::~VtkMprResliceAdapter() = default;
VtkMprResliceAdapter::VtkMprResliceAdapter(VtkMprResliceAdapter&&) noexcept = default;
VtkMprResliceAdapter& VtkMprResliceAdapter::operator=(VtkMprResliceAdapter&&) noexcept = default;

Result<VtkMprResliceResult> VtkMprResliceAdapter::reslice(
    const VolumeData& volume,
    const MprViewState& state,
    const MprSliceRequest& request)
{
#if MEASUREMENT_HAVE_VTK
    const auto parametersResult = buildMprResliceParameters(volume, state, request);
    if (!parametersResult.ok()) {
        return Result<VtkMprResliceResult>::failure(parametersResult.error());
    }

    const bool canReuseSourceVolume = m_impl->sourceVolume != nullptr
        && sameDimensions(m_impl->lastVolumeDimensions, volume.metadata.dimensions)
        && m_impl->lastSeriesUid == volume.seriesUid
        && m_impl->lastStudyUid == volume.studyUid
        && m_impl->lastDataHash == volume.dataHash;
    if (!canReuseSourceVolume) {
        const auto vtkVolumeResult = makeVtkImageData(volume);
        if (!vtkVolumeResult.ok()) {
            return Result<VtkMprResliceResult>::failure(vtkVolumeResult.error());
        }
        m_impl->sourceVolume = vtkVolumeResult.value();
        m_impl->lastVolumeDimensions = volume.metadata.dimensions;
        m_impl->lastSeriesUid = volume.seriesUid;
        m_impl->lastStudyUid = volume.studyUid;
        m_impl->lastDataHash = volume.dataHash;
    }

    const MprResliceParameters& parameters = parametersResult.value();
    const MprSliceFrame& frame = parameters.frame;
    const MprSliceRequest& normalizedRequest = parameters.request;

    const Vec3d originVoxel = patientToVoxel(volume.transform, frame.originPatientMm);
    const Vec3d horizontalVoxelPerMm = patientVectorToVoxelVector(volume.transform, frame.horizontalPatientUnit);
    const Vec3d verticalVoxelPerMm = patientVectorToVoxelVector(volume.transform, frame.verticalPatientUnit);
    const Vec3d normalVoxelPerMm = patientVectorToVoxelVector(volume.transform, frame.normalPatientUnit);

    vtkNew<vtkMatrix4x4> axes;
    axes->Identity();
    axes->SetElement(0, 0, horizontalVoxelPerMm.x);
    axes->SetElement(1, 0, horizontalVoxelPerMm.y);
    axes->SetElement(2, 0, horizontalVoxelPerMm.z);
    axes->SetElement(0, 1, verticalVoxelPerMm.x);
    axes->SetElement(1, 1, verticalVoxelPerMm.y);
    axes->SetElement(2, 1, verticalVoxelPerMm.z);
    axes->SetElement(0, 2, normalVoxelPerMm.x);
    axes->SetElement(1, 2, normalVoxelPerMm.y);
    axes->SetElement(2, 2, normalVoxelPerMm.z);
    axes->SetElement(0, 3, originVoxel.x);
    axes->SetElement(1, 3, originVoxel.y);
    axes->SetElement(2, 3, originVoxel.z);

    vtkNew<vtkImageReslice> reslice;
    reslice->SetInputData(m_impl->sourceVolume);
    reslice->SetOutputDimensionality(2);
    reslice->SetResliceAxes(axes);
    reslice->SetInterpolationModeToLinear();
    reslice->SetOutputScalarType(VTK_SHORT);
    reslice->SetBackgroundLevel(static_cast<double>(volume.metadata.minHu));
    reslice->SetOutputSpacing(normalizedRequest.pixelSpacingMm, normalizedRequest.pixelSpacingMm, 1.0);
    reslice->SetOutputOrigin(
        -0.5 * static_cast<double>(normalizedRequest.outputWidth - 1) * normalizedRequest.pixelSpacingMm,
        -0.5 * static_cast<double>(normalizedRequest.outputHeight - 1) * normalizedRequest.pixelSpacingMm,
        0.0);
    reslice->SetOutputExtent(0, normalizedRequest.outputWidth - 1, 0, normalizedRequest.outputHeight - 1, 0, 0);
    reslice->Update();

    m_impl->lastImage = vtkSmartPointer<vtkImageData>::New();
    m_impl->lastImage->DeepCopy(reslice->GetOutput());

    VtkMprResliceResult result;
    result.image = m_impl->lastImage;
    result.readyToRender = result.image != nullptr;
    result.imageChanged = result.readyToRender;
    result.width = normalizedRequest.outputWidth;
    result.height = normalizedRequest.outputHeight;
    return Result<VtkMprResliceResult>::success(result);
#else
    (void)volume;
    (void)state;
    (void)request;
    return Result<VtkMprResliceResult>::failure(mprError(
        "MPR_VTK_UNAVAILABLE",
        "VTK MPR reslice is unavailable in this build.",
        "Configure with VTK to enable vtkImageReslice."));
#endif
}

}  // namespace measurement
