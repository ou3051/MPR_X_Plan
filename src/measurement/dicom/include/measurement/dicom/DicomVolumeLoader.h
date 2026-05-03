#pragma once

#include "measurement/core/Result.h"
#include "measurement/core/Volume.h"

#include <filesystem>

namespace measurement {

class DicomVolumeLoader {
public:
    [[nodiscard]] Result<VolumeData> loadFolder(const std::filesystem::path& folder) const;
};

}  // namespace measurement
