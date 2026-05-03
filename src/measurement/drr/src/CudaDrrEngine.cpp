#include "measurement/drr/CudaDrrEngine.h"

namespace measurement {

Result<void> CudaDrrEngine::setVolume(const VolumeData& volume)
{
    if (!volume.image) {
        return Result<void>::failure({"CUDA_DRR_VOLUME_EMPTY", "CUDA DRR render requires a volume image.", "", true});
    }
    m_volume = volume;
    return Result<void>::success();
}

Result<DrrImage> CudaDrrEngine::render(const ProjectionParams& params, const DrrRenderSettings& settings) const
{
    (void)params;
    (void)settings;
#if MEASUREMENT_HAVE_CUDA
    return Result<DrrImage>::failure({
        "CUDA_DRR_NOT_IMPLEMENTED",
        "CUDA toolkit is available, but the CUDA DRR kernel is not implemented yet.",
        "",
        true,
    });
#else
    return Result<DrrImage>::failure({
        "CUDA_DRR_DISABLED",
        "CUDA DRR is disabled because CUDA Toolkit was not found or MPR_ENABLE_CUDA_DRR is OFF.",
        "",
        true,
    });
#endif
}

}  // namespace measurement
