#pragma once

#include "measurement/core/Result.h"
#include "measurement/core/Volume.h"
#include "measurement/core/Xray.h"

#include <memory>

namespace measurement {

class CudaDrrEngine {
public:
    CudaDrrEngine();
    ~CudaDrrEngine();

    CudaDrrEngine(const CudaDrrEngine&) = delete;
    CudaDrrEngine& operator=(const CudaDrrEngine&) = delete;
    CudaDrrEngine(CudaDrrEngine&&) noexcept;
    CudaDrrEngine& operator=(CudaDrrEngine&&) noexcept;

    [[nodiscard]] Result<void> setVolume(const VolumeData& volume);
    [[nodiscard]] Result<DrrImage> render(const ProjectionParams& params, const DrrRenderSettings& settings) const;

private:
    struct Impl;

    std::unique_ptr<Impl> m_impl;
    VolumeData m_volume;
};

}  // namespace measurement
