# 公共数据接口 v0.1

文档状态：草案
版本：v0.1
日期：2026-05-04
接口来源：`src/measurement/*/include`

## 1. 接口边界原则

- `measurement_core` 是公共数据模型的唯一来源，不依赖 Qt、VTK、DCMTK、CUDA。
- 外部库类型不得出现在 core public headers 中。
- DICOM 模块输出 `VolumeData`，不暴露 DCMTK 类型。
- MPR 模块使用 `MprViewState` 作为外部状态，不使用 VTK widget 作为业务状态来源。
- DRR 模块使用 `ProjectionParams` 与 `DrrRenderSettings`，CPU 与 CUDA 共享同一输入模型。
- Persistence 模块读写 `.mprproj` 单文件工程包，包内主入口为 `manifest.json`。

## 2. Core 基础类型

```cpp
namespace measurement {

struct Vec3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Size3i {
    int x = 0;
    int y = 0;
    int z = 0;
};

struct Mat4d {
    std::array<double, 16> values{};

    static Mat4d identity();
    [[nodiscard]] double at(int row, int column) const;
    double& at(int row, int column);
};

[[nodiscard]] Vec3d operator+(Vec3d lhs, Vec3d rhs);
[[nodiscard]] Vec3d operator-(Vec3d lhs, Vec3d rhs);
[[nodiscard]] Vec3d operator*(Vec3d vector, double scalar);
[[nodiscard]] Vec3d operator*(double scalar, Vec3d vector);
[[nodiscard]] Vec3d operator/(Vec3d vector, double scalar);

[[nodiscard]] double dot(Vec3d lhs, Vec3d rhs);
[[nodiscard]] Vec3d cross(Vec3d lhs, Vec3d rhs);
[[nodiscard]] double length(Vec3d vector);
[[nodiscard]] Vec3d normalize(Vec3d vector);
[[nodiscard]] bool nearlyEqual(double lhs, double rhs, double tolerance = 1.0e-9);
[[nodiscard]] bool nearlyEqual(Vec3d lhs, Vec3d rhs, double tolerance = 1.0e-9);
[[nodiscard]] Vec3d transformPoint(const Mat4d& matrix, Vec3d point);
[[nodiscard]] Vec3d transformVector(const Mat4d& matrix, Vec3d vector);
[[nodiscard]] Mat4d invertAffine(const Mat4d& matrix);

}  // namespace measurement
```

## 3. Result 与错误模型

```cpp
namespace measurement {

inline constexpr std::string_view kErrorDicomFolderNotFound = "DICOM_FOLDER_NOT_FOUND";
inline constexpr std::string_view kErrorDicomDependencyMissing = "DICOM_DEPENDENCY_MISSING";
inline constexpr std::string_view kErrorDicomEmptyFolder = "DICOM_EMPTY_FOLDER";
inline constexpr std::string_view kErrorDicomNoCtSeries = "DICOM_NO_CT_SERIES";
inline constexpr std::string_view kErrorDicomMultiSeriesUnsupported = "DICOM_MULTI_SERIES_UNSUPPORTED";
inline constexpr std::string_view kErrorDicomMissingTag = "DICOM_MISSING_TAG";
inline constexpr std::string_view kErrorDicomInconsistentGeometry = "DICOM_INCONSISTENT_GEOMETRY";
inline constexpr std::string_view kErrorDicomImageBuildFailed = "DICOM_IMAGE_BUILD_FAILED";

inline constexpr std::string_view kErrorVolumeInvalidMetadata = "VOLUME_INVALID_METADATA";
inline constexpr std::string_view kErrorVolumeTransformNotInvertible = "VOLUME_TRANSFORM_NOT_INVERTIBLE";
inline constexpr std::string_view kErrorVolumeImageSizeMismatch = "VOLUME_IMAGE_SIZE_MISMATCH";

struct ErrorInfo {
    std::string code;
    std::string message;
    std::string detail;
    bool recoverable = true;
};

[[nodiscard]] ErrorInfo makeErrorInfo(
    std::string code,
    std::string message,
    std::string detail = {},
    bool recoverable = true);

template <typename T>
class Result {
public:
    static Result success(T value);
    static Result failure(ErrorInfo error);

    [[nodiscard]] bool ok() const;
    [[nodiscard]] const T& value() const;
    [[nodiscard]] T& value();
    [[nodiscard]] const ErrorInfo& error() const;
};

template <>
class Result<void> {
public:
    static Result success();
    static Result failure(ErrorInfo error);

    [[nodiscard]] bool ok() const;
    [[nodiscard]] const ErrorInfo& error() const;
};

}  // namespace measurement
```

错误约定：

- 跨模块业务错误使用 `Result<T>` 或 `Result<void>` 返回。
- `ErrorInfo::code` 使用稳定机器可读字符串；新增跨模块错误码应优先在 core 中固化为 `kError*` 常量。
- `message` 面向开发和日志，`detail` 保存可选上下文，`recoverable` 标识调用方是否可继续流程。
- DICOM CT 导入当前稳定错误码为 `DICOM_FOLDER_NOT_FOUND`、`DICOM_DEPENDENCY_MISSING`、`DICOM_EMPTY_FOLDER`、`DICOM_NO_CT_SERIES`、`DICOM_MULTI_SERIES_UNSUPPORTED`、`DICOM_MISSING_TAG`、`DICOM_INCONSISTENT_GEOMETRY`、`DICOM_IMAGE_BUILD_FAILED`。

## 4. Volume 与坐标接口

```cpp
namespace measurement {

struct VolumeMetadata {
    Size3i dimensions;
    Vec3d spacingMm;
    Vec3d originPatientMm;
    Vec3d rowDirectionPatient;
    Vec3d columnDirectionPatient;
    Vec3d sliceDirectionPatient;
    double rescaleSlope = 1.0;
    double rescaleIntercept = 0.0;
    int minHu = 0;
    int maxHu = 0;
};

struct VolumeTransform {
    Mat4d voxelToPatient = Mat4d::identity();
    Mat4d patientToVoxel = Mat4d::identity();
    Vec3d boundsMinPatientMm;
    Vec3d boundsMaxPatientMm;
};

class IImageVolume {
public:
    virtual ~IImageVolume() = default;
    [[nodiscard]] virtual Size3i dimensions() const = 0;
    [[nodiscard]] virtual int16_t voxelHu(int i, int j, int k) const = 0;
};

class DenseHuVolume final : public IImageVolume {
public:
    DenseHuVolume(Size3i dimensions, std::vector<int16_t> voxels);

    [[nodiscard]] Size3i dimensions() const override;
    [[nodiscard]] int16_t voxelHu(int i, int j, int k) const override;
    [[nodiscard]] const std::vector<int16_t>& voxels() const;
};

struct VolumeData {
    VolumeMetadata metadata;
    VolumeTransform transform;
    std::shared_ptr<IImageVolume> image;
    std::string sourceFolder;
    std::string seriesUid;
    std::string studyUid;
    std::string dataHash;
};

[[nodiscard]] Result<void> validateVolumeMetadata(const VolumeMetadata& metadata);
[[nodiscard]] Result<std::shared_ptr<DenseHuVolume>> makeDenseHuVolume(Size3i dimensions, std::vector<int16_t> voxels);
[[nodiscard]] Result<VolumeTransform> makeVolumeTransform(const VolumeMetadata& metadata);
[[nodiscard]] Vec3d voxelToPatient(const VolumeTransform& transform, Vec3d voxel);
[[nodiscard]] Vec3d patientToVoxel(const VolumeTransform& transform, Vec3d patient);

}  // namespace measurement
```

坐标约定：

- 业务坐标统一为 DICOM patient coordinate。
- 长度单位统一为 mm。
- `DenseHuVolume` 使用 slice-major 顺序存储 HU：`k * dimensions.x * dimensions.y + j * dimensions.x + i`。
- `validateVolumeMetadata` 至少校验 dimensions、spacing、origin、方向向量、rescale 参数和 HU 范围。
- `VolumeTransform::voxelToPatient` 和 `VolumeTransform::patientToVoxel` 是 voxel/patient 转换唯一入口。

## 5. 器械规划接口

```cpp
namespace measurement {

enum class InstrumentType {
    GuidePin,
    PedicleScrew
};

struct Instrument {
    std::string id;
    InstrumentType type = InstrumentType::GuidePin;
    Vec3d entryPointPatientMm;
    Vec3d directionPatientUnit{0.0, 0.0, 1.0};
    double lengthMm = 0.0;
    double diameterMm = 0.0;
    bool visible = true;
    bool locked = false;
    std::string label;
};

struct InstrumentPatch {
    Vec3d entryPointPatientMm;
    Vec3d directionPatientUnit;
    double lengthMm = 0.0;
    double diameterMm = 0.0;
    bool visible = true;
    bool locked = false;
    std::string label;
};

[[nodiscard]] Vec3d endpointPatientMm(const Instrument& instrument);
[[nodiscard]] Result<void> validateInstrument(const Instrument& instrument);

class SurgicalPlan {
public:
    [[nodiscard]] const std::vector<Instrument>& instruments() const;
    [[nodiscard]] Result<void> addInstrument(Instrument instrument);
    [[nodiscard]] Result<void> updateInstrument(const std::string& id, const InstrumentPatch& patch);
    [[nodiscard]] Result<void> removeInstrument(const std::string& id);
    [[nodiscard]] Instrument* findInstrument(const std::string& id);
    [[nodiscard]] const Instrument* findInstrument(const std::string& id) const;
};

}  // namespace measurement
```

## 6. X 射线与 DRR 接口

```cpp
namespace measurement {

enum class XrayPreset {
    AP,
    LAT,
    Oblique,
    Custom
};

struct ProjectionParams {
    Vec3d sourcePosPatientMm;
    Vec3d detectorCenterPatientMm;
    Vec3d detectorUPatientUnit{1.0, 0.0, 0.0};
    Vec3d detectorVPatientUnit{0.0, 1.0, 0.0};
    double pixelSpacingMm = 0.5;
    int detectorWidth = 512;
    int detectorHeight = 512;
    double primaryAngleDeg = 0.0;
    double secondaryAngleDeg = 0.0;
    double sidMm = 1000.0;
    double sodMm = 700.0;
};

struct XrayView {
    XrayPreset preset = XrayPreset::AP;
    ProjectionParams projection;
    double windowCenter = 0.0;
    double windowWidth = 1.0;
    bool showInstrumentOverlay = true;
};

struct DrrRenderSettings {
    int width = 512;
    int height = 512;
    double stepMm = 0.5;
    bool outputLineIntegral = true;
    double windowCenter = 0.0;
    double windowWidth = 1.0;
    double gamma = 1.0;
};

struct DrrImage {
    int width = 0;
    int height = 0;
    std::vector<float> lineIntegral;
    std::vector<uint16_t> displayImage;
    ProjectionParams projection;
    uint64_t frameId = 0;
};

}  // namespace measurement
```

## 7. 模块服务接口

```cpp
namespace measurement {

class DicomVolumeLoader {
public:
    [[nodiscard]] Result<VolumeData> loadFolder(const std::filesystem::path& folder) const;
};

enum class MprPlane {
    Axial,
    Sagittal,
    Coronal
};

struct MprViewState {
    MprPlane plane = MprPlane::Axial;
    Vec3d crosshairPatientMm;
    double zoom = 1.0;
    Vec3d pan;
    double windowCenterHu = 400.0;
    double windowWidthHu = 2000.0;
};

struct MprSliceImage {
    int width = 0;
    int height = 0;
    std::vector<int16_t> huPixels;
};

class MprResliceEngine {
public:
    [[nodiscard]] Result<MprSliceImage> reslice(const VolumeData& volume, const MprViewState& state) const;
};

[[nodiscard]] Vec3d planeNormalPatient(const VolumeMetadata& metadata, MprPlane plane);

struct MeshData {
    std::vector<Vec3d> vertices;
    std::vector<unsigned int> indices;
};

class InstrumentGeometryBuilder {
public:
    [[nodiscard]] MeshData buildGuidePinMesh(const Instrument& instrument, int radialSegments = 24) const;
    [[nodiscard]] MeshData buildPedicleScrewMesh(const Instrument& instrument, int radialSegments = 24) const;
};

class CpuDrrEngine {
public:
    [[nodiscard]] Result<void> setVolume(const VolumeData& volume);
    [[nodiscard]] Result<DrrImage> render(const ProjectionParams& params, const DrrRenderSettings& settings) const;
};

class CudaDrrEngine {
public:
    [[nodiscard]] Result<void> setVolume(const VolumeData& volume);
    [[nodiscard]] Result<DrrImage> render(const ProjectionParams& params, const DrrRenderSettings& settings) const;
};

struct VtkCameraFrame {
    Vec3d cameraPositionWorld;
    Vec3d focalPointWorld;
    Vec3d viewUpWorld{0.0, 1.0, 0.0};
    Mat4d actorToWorld = Mat4d::identity();
    double sidMm = 1000.0;
};

class VtkPhysicsAdapter {
public:
    [[nodiscard]] Result<ProjectionParams> buildProjectionParams(const VtkCameraFrame& frame) const;
};

struct ProjectManifest {
    std::string schemaVersion = "0.1";
    std::string softwareVersion = "0.1.0";
    std::string dicomSourceFolder;
    std::string studyUid;
    std::string seriesUid;
    std::string dataHash;
    SurgicalPlan plan;
    XrayView xrayView;
};

[[nodiscard]] std::string serializeProjectManifest(const ProjectManifest& manifest);

}  // namespace measurement
```

## 8. 审批关注点

- 本文档中的签名应与 public headers 同步更新。
- 新增 public type、public method 或跨模块错误码时，必须同时更新本文档和单元测试。
- 若后续引入 Qt/VTK/DCMTK/CUDA 类型作为 public API，必须经过架构评审并记录偏离原因。
