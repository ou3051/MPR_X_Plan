#include "measurement/dicom/DicomVolumeLoader.h"

#include <filesystem>

#if MEASUREMENT_HAVE_DCMTK
#include <dcmtk/dcmdata/dctk.h>
#endif

namespace measurement {

Result<VolumeData> DicomVolumeLoader::loadFolder(const std::filesystem::path& folder) const
{
    if (!std::filesystem::exists(folder) || !std::filesystem::is_directory(folder)) {
        return Result<VolumeData>::failure({
            "DICOM_FOLDER_NOT_FOUND",
            "DICOM folder does not exist.",
            folder.string(),
            true,
        });
    }

#if MEASUREMENT_HAVE_DCMTK
    DcmFileFormat format;
    (void)format.getDataset();

    return Result<VolumeData>::failure({
        "DICOM_LOADER_NOT_IMPLEMENTED",
        "DCMTK is available, but the DICOM loader implementation is not complete yet.",
        folder.string(),
        true,
    });
#else
    return Result<VolumeData>::failure({
        "DICOM_DEPENDENCY_MISSING",
        "DCMTK was not found. DICOM import is disabled for this build.",
        "Install DCMTK or fetch the pinned fallback into third_party/dcmtk.",
        true,
    });
#endif
}

}  // namespace measurement
