#pragma once

#include "measurement/core/Result.h"
#include "measurement/core/Volume.h"
#include "measurement/core/Xray.h"

namespace measurement {

class CpuDrrEngine {
public:
    [[nodiscard]] Result<void> setVolume(const VolumeData& volume);
    [[nodiscard]] Result<DrrImage> render(const ProjectionParams& params, const DrrRenderSettings& settings) const;

private:
    VolumeData m_volume;
};

}  // namespace measurement
