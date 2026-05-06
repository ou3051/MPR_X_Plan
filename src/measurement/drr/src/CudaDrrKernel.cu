#include "CudaDrrKernel.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace measurement {
namespace {

struct CudaVec3 {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

struct CudaProjection {
    CudaVec3 source;
    CudaVec3 detectorCenter;
    CudaVec3 detectorU;
    CudaVec3 detectorV;
    float pixelSpacingMm = 1.0F;
    int width = 0;
    int height = 0;
};

struct CudaVolumeTransform {
    float patientToVoxel[12]{};
    CudaVec3 boundsMin;
    CudaVec3 boundsMax;
    int dimX = 0;
    int dimY = 0;
    int dimZ = 0;
};

struct CudaRenderSettings {
    float stepMm = 1.0F;
    float windowCenter = 0.0F;
    float windowWidth = 1.0F;
    float gamma = 1.0F;
    float huOffset = 0.0F;
    float huScale = 1.0F;
};

[[nodiscard]] ErrorInfo cudaError(std::string message, cudaError_t error)
{
    return makeErrorInfo(
        "CUDA_DRR_RUNTIME_ERROR",
        std::move(message),
        cudaGetErrorString(error),
        true);
}

[[nodiscard]] bool dimensionsArePositive(Size3i dimensions)
{
    return dimensions.x > 0 && dimensions.y > 0 && dimensions.z > 0;
}

[[nodiscard]] size_t voxelCount(Size3i dimensions)
{
    return static_cast<size_t>(dimensions.x)
        * static_cast<size_t>(dimensions.y)
        * static_cast<size_t>(dimensions.z);
}

[[nodiscard]] CudaVec3 toCuda(Vec3d value)
{
    return {
        static_cast<float>(value.x),
        static_cast<float>(value.y),
        static_cast<float>(value.z),
    };
}

[[nodiscard]] CudaProjection toCudaProjection(const ProjectionParams& params, const DrrRenderSettings& settings)
{
    return {
        toCuda(params.sourcePosPatientMm),
        toCuda(params.detectorCenterPatientMm),
        toCuda(normalize(params.detectorUPatientUnit)),
        toCuda(normalize(params.detectorVPatientUnit)),
        static_cast<float>(params.pixelSpacingMm),
        settings.width,
        settings.height,
    };
}

[[nodiscard]] CudaRenderSettings toCudaSettings(const DrrRenderSettings& settings)
{
    return {
        static_cast<float>(settings.stepMm),
        static_cast<float>(settings.windowCenter),
        static_cast<float>(settings.windowWidth),
        static_cast<float>(settings.gamma),
        static_cast<float>(settings.huOffset),
        static_cast<float>(settings.huScale),
    };
}

[[nodiscard]] CudaVolumeTransform toCudaTransform(const VolumeData& volume)
{
    CudaVolumeTransform transform;
    const Size3i dims = volume.image->dimensions();
    transform.dimX = dims.x;
    transform.dimY = dims.y;
    transform.dimZ = dims.z;
    transform.boundsMin = toCuda(volume.transform.boundsMinPatientMm);
    transform.boundsMax = toCuda(volume.transform.boundsMaxPatientMm);

    transform.patientToVoxel[0] = static_cast<float>(volume.transform.patientToVoxel.at(0, 0));
    transform.patientToVoxel[1] = static_cast<float>(volume.transform.patientToVoxel.at(0, 1));
    transform.patientToVoxel[2] = static_cast<float>(volume.transform.patientToVoxel.at(0, 2));
    transform.patientToVoxel[3] = static_cast<float>(volume.transform.patientToVoxel.at(0, 3));
    transform.patientToVoxel[4] = static_cast<float>(volume.transform.patientToVoxel.at(1, 0));
    transform.patientToVoxel[5] = static_cast<float>(volume.transform.patientToVoxel.at(1, 1));
    transform.patientToVoxel[6] = static_cast<float>(volume.transform.patientToVoxel.at(1, 2));
    transform.patientToVoxel[7] = static_cast<float>(volume.transform.patientToVoxel.at(1, 3));
    transform.patientToVoxel[8] = static_cast<float>(volume.transform.patientToVoxel.at(2, 0));
    transform.patientToVoxel[9] = static_cast<float>(volume.transform.patientToVoxel.at(2, 1));
    transform.patientToVoxel[10] = static_cast<float>(volume.transform.patientToVoxel.at(2, 2));
    transform.patientToVoxel[11] = static_cast<float>(volume.transform.patientToVoxel.at(2, 3));
    return transform;
}

[[nodiscard]] std::vector<int16_t> copyHuVoxels(const VolumeData& volume)
{
    const Size3i dims = volume.image->dimensions();
    std::vector<int16_t> voxels;
    voxels.reserve(voxelCount(dims));
    for (int k = 0; k < dims.z; ++k) {
        for (int j = 0; j < dims.y; ++j) {
            for (int i = 0; i < dims.x; ++i) {
                voxels.push_back(volume.image->voxelHu(i, j, k));
            }
        }
    }
    return voxels;
}

__device__ CudaVec3 add(CudaVec3 lhs, CudaVec3 rhs)
{
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

__device__ CudaVec3 subtract(CudaVec3 lhs, CudaVec3 rhs)
{
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

__device__ CudaVec3 multiply(CudaVec3 vector, float scalar)
{
    return {vector.x * scalar, vector.y * scalar, vector.z * scalar};
}

__device__ float dot(CudaVec3 lhs, CudaVec3 rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

__device__ float length(CudaVec3 vector)
{
    return sqrtf(dot(vector, vector));
}

__device__ CudaVec3 normalize(CudaVec3 vector)
{
    const float vectorLength = length(vector);
    if (vectorLength <= 0.0F) {
        return {};
    }
    return multiply(vector, 1.0F / vectorLength);
}

__device__ float huToMu(int16_t hu, CudaRenderSettings settings)
{
    const float calibratedHu = static_cast<float>(hu) * settings.huScale + settings.huOffset;
    return fmaxf(calibratedHu + 1000.0F, 0.0F) / 1000.0F;
}

__device__ bool intersectAabb(
    CudaVec3 source,
    CudaVec3 direction,
    CudaVec3 boundsMin,
    CudaVec3 boundsMax,
    float* tEnter,
    float* tExit)
{
    *tEnter = 0.0F;
    *tExit = 3.402823466e+38F;

    const float sourceValues[3] = {source.x, source.y, source.z};
    const float directionValues[3] = {direction.x, direction.y, direction.z};
    const float minValues[3] = {boundsMin.x, boundsMin.y, boundsMin.z};
    const float maxValues[3] = {boundsMax.x, boundsMax.y, boundsMax.z};

    for (int axis = 0; axis < 3; ++axis) {
        if (fabsf(directionValues[axis]) < 1.0e-12F) {
            if (sourceValues[axis] < minValues[axis] || sourceValues[axis] > maxValues[axis]) {
                return false;
            }
            continue;
        }

        float t0 = (minValues[axis] - sourceValues[axis]) / directionValues[axis];
        float t1 = (maxValues[axis] - sourceValues[axis]) / directionValues[axis];
        if (t0 > t1) {
            const float temp = t0;
            t0 = t1;
            t1 = temp;
        }
        *tEnter = fmaxf(*tEnter, t0);
        *tExit = fminf(*tExit, t1);
        if (*tExit < *tEnter) {
            return false;
        }
    }
    return true;
}

__device__ int clampIndex(float value, int upperExclusive)
{
    return min(max(static_cast<int>(llroundf(value)), 0), upperExclusive - 1);
}

__device__ int16_t nearestHu(const int16_t* voxels, CudaVolumeTransform volume, CudaVec3 patient)
{
    const float voxelX = volume.patientToVoxel[0] * patient.x
        + volume.patientToVoxel[1] * patient.y
        + volume.patientToVoxel[2] * patient.z
        + volume.patientToVoxel[3];
    const float voxelY = volume.patientToVoxel[4] * patient.x
        + volume.patientToVoxel[5] * patient.y
        + volume.patientToVoxel[6] * patient.z
        + volume.patientToVoxel[7];
    const float voxelZ = volume.patientToVoxel[8] * patient.x
        + volume.patientToVoxel[9] * patient.y
        + volume.patientToVoxel[10] * patient.z
        + volume.patientToVoxel[11];

    const int i = clampIndex(voxelX, volume.dimX);
    const int j = clampIndex(voxelY, volume.dimY);
    const int k = clampIndex(voxelZ, volume.dimZ);
    const size_t sliceSize = static_cast<size_t>(volume.dimX) * static_cast<size_t>(volume.dimY);
    const size_t index = static_cast<size_t>(k) * sliceSize
        + static_cast<size_t>(j) * static_cast<size_t>(volume.dimX)
        + static_cast<size_t>(i);
    return voxels[index];
}

__device__ uint16_t mapIntegralToDisplay(float integral, CudaRenderSettings settings)
{
    const float lower = settings.windowCenter - settings.windowWidth * 0.5F;
    const float normalized = fminf(fmaxf((integral - lower) / settings.windowWidth, 0.0F), 1.0F);
    const float gammaCorrected = powf(normalized, 1.0F / settings.gamma);
    return static_cast<uint16_t>(fminf(fmaxf(gammaCorrected, 0.0F), 1.0F) * 65535.0F);
}

__global__ void drrRaycastKernel(
    const int16_t* voxels,
    CudaVolumeTransform volume,
    CudaProjection projection,
    CudaRenderSettings settings,
    float* lineIntegral,
    uint16_t* displayImage)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= projection.width || y >= projection.height) {
        return;
    }

    const float offsetU = (static_cast<float>(x) + 0.5F - static_cast<float>(projection.width) * 0.5F) * projection.pixelSpacingMm;
    const float offsetV = (static_cast<float>(y) + 0.5F - static_cast<float>(projection.height) * 0.5F) * projection.pixelSpacingMm;
    const CudaVec3 pixelPos = add(
        add(projection.detectorCenter, multiply(projection.detectorU, offsetU)),
        multiply(projection.detectorV, offsetV));
    const CudaVec3 rayDir = normalize(subtract(pixelPos, projection.source));

    float tEnter = 0.0F;
    float tExit = 0.0F;
    float integral = 0.0F;
    if (intersectAabb(projection.source, rayDir, volume.boundsMin, volume.boundsMax, &tEnter, &tExit)) {
        for (float t = tEnter; t <= tExit; t += settings.stepMm) {
            const CudaVec3 sample = add(projection.source, multiply(rayDir, t));
            integral += huToMu(nearestHu(voxels, volume, sample), settings) * settings.stepMm;
        }
    }

    const size_t index = static_cast<size_t>(y) * static_cast<size_t>(projection.width) + static_cast<size_t>(x);
    lineIntegral[index] = integral;
    displayImage[index] = mapIntegralToDisplay(integral, settings);
}

}  // namespace

struct CudaDrrDeviceVolume {
    int16_t* voxels = nullptr;
    size_t voxelCount = 0;
    CudaVolumeTransform transform;
};

Result<CudaDrrDeviceVolume*> createCudaDrrDeviceVolume(const VolumeData& volume)
{
    if (!volume.image) {
        return Result<CudaDrrDeviceVolume*>::failure({"CUDA_DRR_VOLUME_EMPTY", "CUDA DRR render requires a volume image.", "", true});
    }
    const Size3i dims = volume.image->dimensions();
    if (!dimensionsArePositive(dims)) {
        return Result<CudaDrrDeviceVolume*>::failure({"CUDA_DRR_VOLUME_EMPTY", "CUDA DRR render requires a non-empty volume image.", "", true});
    }

    int deviceCount = 0;
    cudaError_t error = cudaGetDeviceCount(&deviceCount);
    if (error != cudaSuccess) {
        return Result<CudaDrrDeviceVolume*>::failure(cudaError("CUDA device query failed.", error));
    }
    if (deviceCount <= 0) {
        return Result<CudaDrrDeviceVolume*>::failure(makeErrorInfo(
            "CUDA_DRR_NO_DEVICE",
            "CUDA DRR requires a CUDA-capable GPU.",
            "",
            true));
    }

    const std::vector<int16_t> hostVoxels = copyHuVoxels(volume);
    const size_t voxelBytes = hostVoxels.size() * sizeof(int16_t);

    int16_t* deviceVoxels = nullptr;
    error = cudaMalloc(reinterpret_cast<void**>(&deviceVoxels), voxelBytes);
    if (error != cudaSuccess) {
        return Result<CudaDrrDeviceVolume*>::failure(cudaError("CUDA volume allocation failed.", error));
    }

    error = cudaMemcpy(deviceVoxels, hostVoxels.data(), voxelBytes, cudaMemcpyHostToDevice);
    if (error != cudaSuccess) {
        cudaFree(deviceVoxels);
        return Result<CudaDrrDeviceVolume*>::failure(cudaError("CUDA volume upload failed.", error));
    }

    // The cached transform is the fixed patient/physics -> voxel mapping. Camera and actor
    // updates should already be converted to patient-space ProjectionParams before rendering.
    auto* deviceVolume = new CudaDrrDeviceVolume;
    deviceVolume->voxels = deviceVoxels;
    deviceVolume->voxelCount = hostVoxels.size();
    deviceVolume->transform = toCudaTransform(volume);
    return Result<CudaDrrDeviceVolume*>::success(deviceVolume);
}

void destroyCudaDrrDeviceVolume(CudaDrrDeviceVolume* deviceVolume) noexcept
{
    if (deviceVolume == nullptr) {
        return;
    }
    cudaFree(deviceVolume->voxels);
    delete deviceVolume;
}

Result<DrrImage> renderCudaDrr(
    const CudaDrrDeviceVolume& deviceVolume,
    const ProjectionParams& params,
    const DrrRenderSettings& settings)
{
    const size_t pixelCount = static_cast<size_t>(settings.width) * static_cast<size_t>(settings.height);

    float* deviceLineIntegral = nullptr;
    uint16_t* deviceDisplayImage = nullptr;

    cudaError_t error = cudaMalloc(reinterpret_cast<void**>(&deviceLineIntegral), pixelCount * sizeof(float));
    if (error != cudaSuccess) {
        return Result<DrrImage>::failure(cudaError("CUDA line integral allocation failed.", error));
    }
    error = cudaMalloc(reinterpret_cast<void**>(&deviceDisplayImage), pixelCount * sizeof(uint16_t));
    if (error != cudaSuccess) {
        cudaFree(deviceLineIntegral);
        return Result<DrrImage>::failure(cudaError("CUDA display image allocation failed.", error));
    }

    const dim3 block(16, 16);
    const dim3 grid(
        static_cast<unsigned int>((settings.width + static_cast<int>(block.x) - 1) / static_cast<int>(block.x)),
        static_cast<unsigned int>((settings.height + static_cast<int>(block.y) - 1) / static_cast<int>(block.y)));
    drrRaycastKernel<<<grid, block>>>(
        deviceVolume.voxels,
        deviceVolume.transform,
        toCudaProjection(params, settings),
        toCudaSettings(settings),
        deviceLineIntegral,
        deviceDisplayImage);
    error = cudaGetLastError();
    if (error == cudaSuccess) {
        error = cudaDeviceSynchronize();
    }
    if (error != cudaSuccess) {
        cudaFree(deviceDisplayImage);
        cudaFree(deviceLineIntegral);
        return Result<DrrImage>::failure(cudaError("CUDA DRR kernel execution failed.", error));
    }

    DrrImage image;
    image.width = settings.width;
    image.height = settings.height;
    image.projection = params;
    if (settings.outputLineIntegral) {
        image.lineIntegral.resize(pixelCount, 0.0F);
        error = cudaMemcpy(image.lineIntegral.data(), deviceLineIntegral, pixelCount * sizeof(float), cudaMemcpyDeviceToHost);
        if (error != cudaSuccess) {
            cudaFree(deviceDisplayImage);
            cudaFree(deviceLineIntegral);
            return Result<DrrImage>::failure(cudaError("CUDA line integral download failed.", error));
        }
    }

    image.displayImage.resize(pixelCount, 0U);
    error = cudaMemcpy(image.displayImage.data(), deviceDisplayImage, pixelCount * sizeof(uint16_t), cudaMemcpyDeviceToHost);

    cudaFree(deviceDisplayImage);
    cudaFree(deviceLineIntegral);

    if (error != cudaSuccess) {
        return Result<DrrImage>::failure(cudaError("CUDA display image download failed.", error));
    }

    return Result<DrrImage>::success(std::move(image));
}

}  // namespace measurement
