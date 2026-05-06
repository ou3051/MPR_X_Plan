#pragma once

#include "measurement/core/Result.h"
#include "measurement/core/Volume.h"
#include "measurement/mpr/MprResliceEngine.h"

#include <memory>

class vtkImageData;

namespace measurement {

struct VtkMprResliceResult {
    vtkImageData* image = nullptr;
    bool readyToRender = false;
    bool imageChanged = false;
    int width = 0;
    int height = 0;
};

class VtkMprResliceAdapter {
public:
    VtkMprResliceAdapter();
    ~VtkMprResliceAdapter();
    VtkMprResliceAdapter(VtkMprResliceAdapter&&) noexcept;
    VtkMprResliceAdapter& operator=(VtkMprResliceAdapter&&) noexcept;
    VtkMprResliceAdapter(const VtkMprResliceAdapter&) = delete;
    VtkMprResliceAdapter& operator=(const VtkMprResliceAdapter&) = delete;

    [[nodiscard]] Result<VtkMprResliceResult> reslice(
        const VolumeData& volume,
        const MprViewState& state,
        const MprSliceRequest& request);

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

}  // namespace measurement
