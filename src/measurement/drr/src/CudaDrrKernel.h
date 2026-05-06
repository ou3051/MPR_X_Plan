#pragma once

#include "measurement/core/Result.h"
#include "measurement/core/Volume.h"
#include "measurement/core/Xray.h"

namespace measurement {

struct CudaDrrDeviceVolume;

[[nodiscard]] Result<CudaDrrDeviceVolume*> createCudaDrrDeviceVolume(const VolumeData& volume);
void destroyCudaDrrDeviceVolume(CudaDrrDeviceVolume* deviceVolume) noexcept;

[[nodiscard]] Result<DrrImage> renderCudaDrr(
    const CudaDrrDeviceVolume& deviceVolume,
    const ProjectionParams& params,
    const DrrRenderSettings& settings);

}  // namespace measurement
