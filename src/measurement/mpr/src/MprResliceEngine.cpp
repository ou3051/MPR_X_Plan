#include "measurement/mpr/MprResliceEngine.h"

namespace measurement {

Vec3d planeNormalPatient(const VolumeMetadata& metadata, MprPlane plane)
{
    switch (plane) {
    case MprPlane::Axial:
        return normalize(metadata.sliceDirectionPatient);
    case MprPlane::Sagittal:
        return normalize(metadata.rowDirectionPatient);
    case MprPlane::Coronal:
        return normalize(metadata.columnDirectionPatient);
    }
    return {};
}

Result<MprSliceImage> MprResliceEngine::reslice(const VolumeData& volume, const MprViewState& state) const
{
    (void)state;
    if (!volume.image) {
        return Result<MprSliceImage>::failure({
            "MPR_VOLUME_EMPTY",
            "MPR reslice requires a volume image.",
            "",
            true,
        });
    }

    return Result<MprSliceImage>::failure({
        "MPR_RESLICE_ADAPTER_REQUIRED",
        "MPR pixel resampling is provided by the VTK vtkImageReslice adapter.",
        "Core MPR state is available, but VTK-backed reslice is not implemented in this target.",
        true,
    });
}

}  // namespace measurement
