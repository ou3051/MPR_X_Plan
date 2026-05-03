# 骨科手术规划仿真软件 SAD 详细设计 v0.1

文档状态：草案
版本：v0.1
日期：2026-05-03
关联文档：PRD v0.1、SAD 概要设计 v0.1
设计范围：DICOM、坐标、MPR、3D、器械、DRR、工程保存、测试框架

## 1. 文档目的

本文档定义 v0.1 的详细设计，说明每个关键功能如何搭建、核心类如何协作、输入输出如何约束、异常如何处理、测试如何覆盖。

本文档中的类名和接口是开发基线。实际实现时可按 C++/Qt/VTK 约束微调，但不得改变核心依赖方向：

```text
UI -> Application -> Domain
Visualization -> Domain + Algorithm
Algorithm -> Domain-compatible data
Persistence -> Domain
Tests -> Domain + Algorithm + Persistence
```

## 2. 工程结构设计

### 2.1 目录结构

建议 v0.1 工程结构：

```text
src/
  app/
    main.cpp
    MainWindow.h
    MainWindow.cpp
    panels/
    widgets/
  measurement/
    core/
      include/measurement/core/
      src/
    dicom/
      include/measurement/dicom/
      src/
    mpr/
      include/measurement/mpr/
      src/
    planning/
      include/measurement/planning/
      src/
    drr/
      include/measurement/drr/
      src/
    vtk_adapter/
      include/measurement/vtk/
      src/
    qt_adapter/
      include/measurement/qt/
      src/
    persistence/
      include/measurement/persistence/
      src/
tests/
  unit/
  functional/
  validation/
docs/
```

### 2.2 构建目标

建议 CMake targets：

```text
measurement_core
measurement_dicom
measurement_mpr
measurement_planning
measurement_drr
measurement_vtk_adapter
measurement_persistence
measurement_qt_adapter
measurement_app
measurement_unit_tests
measurement_validation_tests
```

确认技术选型：

- DICOM 解析库：DCMTK。
- 实时 GPU DRR：CUDA。
- MPR 核心：`vtkImageReslice + 外部控制器`。
- 工程文件：单文件工程包，包内包含 manifest JSON。

依赖方向：

```text
core <- dicom
core <- mpr
core <- planning
core <- drr
core + planning + mpr + drr <- vtk_adapter
core + planning + persistence <- qt_adapter
all adapters <- app
```

## 3. Core Domain 详细设计

### 3.1 基础数据类型

```cpp
struct Vec3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Mat4d {
    double m[16] = {};
};

struct Size3i {
    int x = 0;
    int y = 0;
    int z = 0;
};

struct ValidationIssue {
    std::string code;
    std::string message;
    Severity severity;
};
```

设计要求：

- `Vec3d` 和 `Mat4d` 用于 domain 和 algorithm 层。
- VTK、Qt、CUDA 类型不得出现在 core domain 头文件中。
- 所有长度单位默认是毫米。

### 3.2 VolumeData

```cpp
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
    Mat4d voxelToPatient;
    Mat4d patientToVoxel;
    Vec3d boundsMinPatientMm;
    Vec3d boundsMaxPatientMm;
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
```

`IImageVolume` 是图像体数据抽象接口：

```cpp
class IImageVolume {
public:
    virtual ~IImageVolume() = default;
    virtual Size3i dimensions() const = 0;
    virtual int16_t voxelHu(int i, int j, int k) const = 0;
};
```

VTK 实现可封装为：

```cpp
class VtkImageVolume final : public IImageVolume {
public:
    explicit VtkImageVolume(vtkSmartPointer<vtkImageData> image);
    vtkImageData* vtkImage() const;
    Size3i dimensions() const override;
    int16_t voxelHu(int i, int j, int k) const override;
};
```

### 3.3 Instrument

```cpp
enum class InstrumentType {
    GuidePin,
    PedicleScrew
};

struct Instrument {
    std::string id;
    InstrumentType type;
    Vec3d entryPointPatientMm;
    Vec3d directionPatientUnit;
    double lengthMm = 0.0;
    double diameterMm = 0.0;
    bool visible = true;
    bool locked = false;
    std::string label;
};
```

约束：

- `directionPatientUnit` 必须是单位向量。
- `lengthMm > 0`。
- `diameterMm > 0`。
- endpoint 由 `entryPointPatientMm + directionPatientUnit * lengthMm` 计算，不重复保存。

### 3.4 SurgicalPlan

```cpp
class SurgicalPlan {
public:
    const std::vector<Instrument>& instruments() const;
    Result<void> addInstrument(Instrument instrument);
    Result<void> updateInstrument(const std::string& id, const InstrumentPatch& patch);
    Result<void> removeInstrument(const std::string& id);
    Instrument* findInstrument(const std::string& id);

private:
    std::vector<Instrument> m_instruments;
};
```

设计要求：

- 所有修改必须走 `add/update/remove`。
- 修改时执行参数校验。
- Application Layer 负责发出 domain changed 事件。

### 3.5 XrayView

```cpp
enum class XrayPreset {
    AP,
    LAT,
    Oblique,
    Custom
};

struct ProjectionParams {
    Vec3d sourcePosPatientMm;
    Vec3d detectorCenterPatientMm;
    Vec3d detectorUPatientUnit;
    Vec3d detectorVPatientUnit;
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
```

## 4. DICOM 导入详细设计

v0.1 DICOM 导入模块基于 DCMTK 实现。DCMTK 负责 DICOM 文件识别、tag 读取、CT 序列校验、像素数据读取和 rescale 信息提取。VTK 不作为 DICOM 解析库，只接收导入模块构建完成后的 `vtkImageData`。

### 4.1 类接口

```cpp
class DicomVolumeLoader {
public:
    Result<VolumeData> loadFolder(const std::filesystem::path& folder);

private:
    Result<std::vector<DicomSliceInfo>> scanFolder(const std::filesystem::path& folder);
    Result<std::vector<DicomSliceInfo>> selectSingleCtSeries(std::vector<DicomSliceInfo> slices);
    Result<void> validateSlices(const std::vector<DicomSliceInfo>& slices);
    Result<vtkSmartPointer<vtkImageData>> buildImageData(const std::vector<DicomSliceInfo>& slices);
    VolumeMetadata buildMetadata(const std::vector<DicomSliceInfo>& slices) const;
    VolumeTransform buildTransform(const VolumeMetadata& metadata) const;
};
```

### 4.2 处理流程

```text
1. scanFolder
   - 遍历文件夹。
   - 读取 DICOM header。
   - 过滤 Modality != CT 的文件。
   - 提取 SeriesInstanceUID、SOPInstanceUID、IPP、IOP、PixelSpacing、SliceThickness、RescaleSlope、RescaleIntercept。

2. selectSingleCtSeries
   - v0.1 不做多序列选择。
   - 如果只有一个 CT series，接受。
   - 如果多个 CT series，返回明确错误或按配置选择 slice 数最多的序列。
   - 默认策略建议为拒绝多序列，避免审批级场景下隐式选择。

3. validateSlices
   - 检查 IOP 一致。
   - 检查 PixelSpacing 一致。
   - 检查 slice direction 可由 row x column 计算。
   - 按 dot(IPP, sliceDirection) 排序。
   - 检查层间距稳定。
   - 检查 dimensions 一致。

4. buildImageData
   - 按排序结果填充 3D volume。
   - 应用 HU = raw * slope + intercept。
   - 输出 int16 或 float volume。

5. buildTransform
   - 构建 voxel index 到 patient coordinate 的矩阵。
   - 构建逆矩阵。
   - 计算 patient bounds。
```

### 4.3 错误码

| 错误码 | 含义 |
| --- | --- |
| DICOM_EMPTY_FOLDER | 文件夹为空或无可读文件 |
| DICOM_NO_CT_SERIES | 未找到 CT 序列 |
| DICOM_MULTIPLE_SERIES | 找到多个 CT 序列，v0.1 不自动选择 |
| DICOM_MISSING_TAG | 缺少必要 tag |
| DICOM_INCONSISTENT_GEOMETRY | slice 几何不一致 |
| DICOM_IMAGE_BUILD_FAILED | 体数据构建失败 |

### 4.4 测试点

- FR-DICOM-001：合法 CT 文件夹可导入。
- FR-DICOM-002：spacing、origin、direction、HU 与 phantom 预期一致。
- FR-DICOM-003：空文件夹和非 DICOM 文件夹失败且不崩溃。
- FR-DICOM-004：多序列输入按策略拒绝并提示。

## 5. 坐标系统详细设计

### 5.1 DICOM 矩阵构建

由 DICOM metadata 构建：

```text
R = rowDirectionPatient
C = columnDirectionPatient
N = normalize(cross(R, C))
S = spacing = (sx, sy, sz)
O = originPatientMm
```

矩阵：

```text
voxelToPatient =
[ R.x*sx  C.x*sy  N.x*sz  O.x ]
[ R.y*sx  C.y*sy  N.y*sz  O.y ]
[ R.z*sx  C.z*sy  N.z*sz  O.z ]
[ 0       0       0       1   ]
```

坐标转换：

```cpp
Vec3d voxelToPatient(const VolumeTransform& t, Vec3d ijk);
Vec3d patientToVoxel(const VolumeTransform& t, Vec3d patient);
```

### 5.2 VTK 集成约束

VTK 中可能存在：

```text
M_actor: patient/physics -> VTK world
M_actor_inv: VTK world -> patient/physics
```

规则：

- domain 不保存 `M_actor`。
- 如果用户只旋转相机，`M_actor` 保持 identity。
- v0.1 允许系统通过设置矩阵非交互式移动 CT actor，常见来源包括配准预留、测试姿态或工程加载后的重定位。
- 用户不直接拖拽 CT actor；任何 CT actor 矩阵变化必须通过受控接口进入。
- `VtkPhysicsAdapter` 必须用 `M_actor_inv` 将 camera 转回 patient/physics。
- DRR engine 永远只接收 patient/physics coordinate。

### 5.3 详细测试点

- 单位矩阵方向的 voxel-patient 往返。
- 非等距 spacing 的往返。
- 非 identity IOP 的往返。
- `voxelToPatient(patientToVoxel(p))` 误差小于阈值。
- `patientToVoxel(voxelToPatient(i))` 误差小于阈值。

## 6. MPR 详细设计

### 6.1 实现方案选择

MPR 核心采用 `vtkImageReslice + 外部控制器`，不采用 `vtkResliceCursor + vtkResliceCursorWidget` 作为 v0.1 核心方案。

选择理由：

- `vtkImageReslice` 可以由 `MprViewState` 完全外部驱动，符合 domain model 作为单一事实来源的原则。
- 三视图十字线、切面位置、窗宽窗位、器械 overlay 均可由 application/controller 明确控制和测试。
- `vtkResliceCursorWidget` 内部维护交互状态，容易让 VTK widget 状态和 domain 状态出现双源状态。
- 审批级软件需要可验证的坐标转换和切面同步，外部控制更容易写单元测试和功能测试。
- 后续如需任意斜切，可在现有 reslice plane 模型上扩展，而不是重写状态同步体系。

`vtkResliceCursorWidget` 可作为调研、原型或未来斜切交互参考，但不得成为核心规划状态来源。

### 6.2 类接口

```cpp
enum class MprPlane {
    Axial,
    Sagittal,
    Coronal
};

struct MprViewState {
    MprPlane plane;
    Vec3d crosshairPatientMm;
    double zoom = 1.0;
    Vec3d pan;
    double windowCenterHu = 400.0;
    double windowWidthHu = 2000.0;
};

class MprResliceEngine {
public:
    Result<MprSliceImage> reslice(const VolumeData& volume, const MprViewState& state);
    Result<Vec3d> screenToPatient(const MprViewState& state, ScreenPoint point);
    Result<ScreenPoint> patientToScreen(const MprViewState& state, Vec3d patient);
};
```

### 6.3 vtkImageReslice 搭建逻辑

每个 MPR 视图拥有独立的 `vtkImageReslice`，但共享同一份 `VolumeData` 和全局 `crosshairPatientMm`。

```text
1. MprViewAdapter 接收 VolumeData。
2. 为 Axial/Sagittal/Coronal 各创建一个 vtkImageReslice。
3. 根据 MprViewState 构建 reslice axes。
4. vtkImageReslice 从 vtkImageData 中重采样输出 2D slice。
5. 输出 slice 进入 vtkImageMapToWindowLevelColors。
6. 2D renderer 绘制切片、十字线、器械 overlay 和标尺。
```

`vtkImageReslice` 只负责采样，不拥有十字线、切面选择和器械状态。

### 6.4 MPR 切面定义

Axial：

```text
normal = sliceDirectionPatient
u = rowDirectionPatient
v = columnDirectionPatient
```

Sagittal：

```text
normal = rowDirectionPatient
u = columnDirectionPatient
v = sliceDirectionPatient
```

Coronal：

```text
normal = columnDirectionPatient
u = rowDirectionPatient
v = sliceDirectionPatient
```

具体方向可根据医学显示习惯进行翻转，但翻转必须在显示层处理，不改变 domain 坐标。

### 6.5 交互流程

滚轮切片：

```text
1. 用户滚轮。
2. MprViewAdapter 根据当前 plane normal 计算 delta。
3. 新 crosshair = old crosshair + normal * deltaMm。
4. ViewSyncController 更新全局 crosshair。
5. 三个 MPR 视图重采样。
6. 3D 视图更新切面位置。
```

十字线拖拽：

```text
1. 用户在某 MPR 视图拖拽。
2. screenToPatient 计算患者坐标。
3. ViewSyncController 更新 crosshair。
4. 所有视图刷新。
```

### 6.6 器械 overlay

MPR 上显示器械的方法：

```text
1. 对每个可见 instrument 计算中心线段 entry -> endpoint。
2. 与当前 slice plane 求交。
3. 若相交，绘制圆形截面或中心点。
4. 若线段近似平行于切面，绘制投影线段。
5. 选中器械时显示 entry、endpoint 和中心线。
```

### 6.7 测试点

- FR-MPR-001：三正交切面方向正确。
- FR-MPR-002：十字线同步。
- FR-MPR-003：滚轮切片后 crosshair 沿法向移动。
- FR-MPR-004：窗宽窗位映射正确。
- FR-MPR-005：HU 查询与原始体素一致。

## 7. 3D 显示详细设计

### 7.1 类接口

```cpp
class ThreeDViewAdapter {
public:
    void setVolume(const VolumeData& volume);
    void setPlan(const SurgicalPlan& plan);
    void setCrosshair(Vec3d patientMm);
    void setXrayView(const XrayView& view);
    void refresh();

private:
    void rebuildVolumeActor();
    void rebuildInstrumentActors();
    void updateMprPlaneActors();
    void updateXrayCameraActors();
};
```

### 7.2 Volume 显示策略

v0.1 支持两种实现路线：

- 体渲染：基于 `vtkGPUVolumeRayCastMapper`。
- 骨表面：基于阈值和 marching cubes。

优先建议：

```text
M1 使用体渲染快速显示。
M2 增加骨阈值表面，便于器械空间关系观察。
```

### 7.3 器械 actor 构建

`InstrumentActorFactory`：

```cpp
class InstrumentActorFactory {
public:
    vtkSmartPointer<vtkActor> createActor(const Instrument& instrument) const;
    vtkSmartPointer<vtkActor> createCenterLineActor(const Instrument& instrument) const;
    vtkSmartPointer<vtkActor> createHandleActor(const Instrument& instrument) const;
};
```

导针：

- 圆柱体。
- 半径 `diameterMm / 2`。
- 高度 `lengthMm`。
- 中心点 `entry + direction * length / 2`。
- 旋转：将局部 Z 轴对齐到 `directionPatientUnit`。

椎弓根螺钉：

- v0.1 采用简化圆柱杆体。
- 可增加头部简化圆柱或球冠。
- 螺纹不作为几何精度要求，可用纹理或浅色环线表示。

### 7.4 测试点

- FR-3D-001：导入后可见 3D volume 或 surface。
- FR-3D-002：新增器械后 actor 位置正确。
- FR-3D-003：相机交互不改变 instrument domain 数据。
- FR-3D-004：对象可见性开关生效。

## 8. 器械规划详细设计

### 8.1 类接口

```cpp
class InstrumentController {
public:
    Result<std::string> createGuidePin(Vec3d entryPatientMm, Vec3d directionPatient, double lengthMm, double diameterMm);
    Result<std::string> createPedicleScrew(Vec3d entryPatientMm, Vec3d directionPatient, double lengthMm, double diameterMm);
    Result<void> updateInstrument(const std::string& id, InstrumentPatch patch);
    Result<void> removeInstrument(const std::string& id);
    Result<void> setVisible(const std::string& id, bool visible);
    Result<void> setLocked(const std::string& id, bool locked);
};

class InstrumentValidator {
public:
    std::vector<ValidationIssue> validate(const Instrument& instrument, const VolumeData* volume) const;
};

class InstrumentGeometryBuilder {
public:
    MeshData buildGuidePinMesh(const Instrument& instrument) const;
    MeshData buildPedicleScrewMesh(const Instrument& instrument) const;
};
```

### 8.2 创建逻辑

```text
1. 用户选择工具。
2. 用户在 MPR 或 3D 中选择 entry。
3. 用户选择 target 或拖拽方向。
4. direction = normalize(target - entry)。
5. 如果用户输入 length，则 endpoint = entry + direction * length。
6. 如果用户未输入 length，则 length = distance(entry, target)。
7. 校验 length 和 diameter。
8. 创建 Instrument。
9. 添加到 SurgicalPlan。
10. 发布 planChanged。
```

### 8.3 参数边界

初始建议：

| 参数 | 建议范围 |
| --- | --- |
| 导针直径 | 0.5 mm 到 5.0 mm |
| 导针长度 | 10 mm 到 300 mm |
| 螺钉直径 | 2.0 mm 到 12.0 mm |
| 螺钉长度 | 10 mm 到 150 mm |

这些范围应作为配置项，不硬编码到 UI。

### 8.4 编辑逻辑

属性面板编辑：

```text
1. 用户修改 diameter 或 length。
2. Qt panel 生成 InstrumentPatch。
3. InstrumentController 校验 patch。
4. SurgicalPlan 更新对象。
5. ViewSyncController 通知 MPR/3D/Xray 刷新。
```

拖拽端点：

```text
1. 用户拖拽 endpoint handle。
2. Visualization 层反算 patient coordinate。
3. direction = normalize(newEndpoint - entry)。
4. length = distance(newEndpoint, entry)。
5. 通过 InstrumentController 更新。
```

### 8.5 测试点

- FR-INS-001：创建导针。
- FR-INS-002：创建螺钉。
- FR-INS-003：entry/direction/length/diameter 更新后视图同步。
- FR-INS-004：保存加载后坐标不漂移。
- FR-INS-005：属性编辑生效。
- FR-INS-006：选择、隐藏、锁定、删除。
- FR-INS-007：中心线和端点可显示。

## 9. DRR 详细设计

本章节吸收现有 DRR 参考设计中的核心思想：

- X 射线几何用 `ProjectionParams` 表达。
- DRR 基于 Beer-Lambert 线积分。
- CPU reference 与 CUDA realtime 共享同一几何模型。
- VTK camera 与物理仿真之间通过 `VtkPhysicsAdapter` 进行坐标同步。
- CT 在 physics/patient coordinate 中视为静止体数据。

### 9.1 DRR 数据结构

```cpp
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

struct VolumeGpuTransform {
    float patientToVoxel[12];
    float bboxMinPatientMm[3];
    float bboxMaxPatientMm[3];
    int dims[3];
    float spacingMm[3];
};
```

### 9.2 DRR 引擎接口

```cpp
class IDrrEngine {
public:
    virtual ~IDrrEngine() = default;
    virtual Result<void> setVolume(const VolumeData& volume) = 0;
    virtual Result<DrrImage> render(const ProjectionParams& params, const DrrRenderSettings& settings) = 0;
};

class CpuDrrEngine final : public IDrrEngine {
public:
    Result<void> setVolume(const VolumeData& volume) override;
    Result<DrrImage> render(const ProjectionParams& params, const DrrRenderSettings& settings) override;
};

class CudaDrrEngine final : public IDrrEngine {
public:
    Result<void> setVolume(const VolumeData& volume) override;
    Result<DrrImage> render(const ProjectionParams& params, const DrrRenderSettings& settings) override;
    void renderAsync(const ProjectionParams& params, const DrrRenderSettings& settings, DrrCallback callback);
};
```

### 9.3 投影几何

每个 detector pixel 对应一个 patient coordinate 中的物理点：

```text
pixelPos =
    detectorCenter
  + (u - width * 0.5) * pixelSpacing * detectorU
  + (v - height * 0.5) * pixelSpacing * detectorV
```

射线：

```text
rayDir = normalize(pixelPos - sourcePos)
r(t) = sourcePos + t * rayDir
```

与 CT patient-space AABB 求交：

```text
tEnter, tExit = intersectAabb(sourcePos, rayDir, boundsMin, boundsMax)
```

积分：

```text
P(u, v) = sum(mu(r(t)) * stepMm)
```

显示：

```text
I(u, v) = exp(-P(u, v))
```

v0.1 可直接输出 line integral，并使用窗宽窗位映射显示。

### 9.4 HU 到衰减系数

初始模型：

```text
mu = max(HU + 1000, 0) / 1000
```

该模型仅用于 v0.1 几何验证和相对灰度显示。后续可引入材料表：

```cpp
class MaterialTable {
public:
    float huToMu(float hu) const;
};
```

### 9.5 CPU DRR 搭建逻辑

```text
1. 输入 VolumeData、ProjectionParams、DrrRenderSettings。
2. 校验 detector size、pixel spacing、SID、source/detector basis。
3. 对每个 detector pixel：
   3.1 计算 pixelPos。
   3.2 计算 rayDir。
   3.3 与 volume patient bounds 求交。
   3.4 从 tEnter 到 tExit 按 stepMm 采样。
   3.5 patientToVoxel 转换采样点。
   3.6 三线性插值 HU。
   3.7 HU 转 mu。
   3.8 累加 line integral。
4. 后处理生成 displayImage。
5. 返回 DrrImage。
```

CPU 实现必须确定性，不使用随机噪声。

### 9.6 CUDA DRR 搭建逻辑

CUDA 初始化：

```text
1. setVolume 时上传 volume 到 3D texture。
2. 上传 VolumeGpuTransform 到 constant memory 或 uniform buffer。
3. 分配 lineIntegral output buffer。
4. 分配 display image buffer。
```

每帧渲染：

```text
1. 上传 ProjectionParams。
2. 启动 drr_raycasting kernel。
3. 启动 postprocess_lut kernel。
4. 异步拷贝 display image 或映射到 VTK/Qt texture。
5. 回调 XrayViewAdapter。
```

核函数概念：

```cpp
kernel drr_raycasting(volumeTexture, outputLineIntegral, projection, volumeTransform) {
    pixel = thread id;
    pixelPos = detectorCenter + pixelOffsetU + pixelOffsetV;
    rayDir = normalize(pixelPos - sourcePos);
    if (!intersectAabb(sourcePos, rayDir, bbox)) {
        output[pixel] = 0;
        return;
    }
    for (t = tEnter; t < tExit; t += stepMm) {
        pPatient = sourcePos + t * rayDir;
        pVoxel = patientToVoxel * pPatient;
        hu = tex3D(volumeTexture, pVoxel + 0.5);
        mu = huToMu(hu);
        integral += mu * stepMm;
    }
    output[pixel] = integral;
}
```

### 9.7 VtkPhysicsAdapter 详细设计

职责：

```text
VTK camera/actor state -> ProjectionParams in patient coordinate
ProjectionParams in patient coordinate -> VTK camera state
```

接口：

```cpp
class VtkPhysicsAdapter : public QObject {
    Q_OBJECT
public:
    VtkPhysicsAdapter(vtkCamera* camera, vtkVolume* ctVolume, QObject* parent = nullptr);

    void onVolumeLoaded(const VolumeData& volume);
    void setSidMm(double sidMm);
    void setIsocenterPatientMm(Vec3d isocenter);

    ProjectionParams buildProjectionParams() const;
    void applyToVtk(const ProjectionParams& params);

signals:
    void paramsChanged(const ProjectionParams& params);

private:
    void installObservers();
    void onCameraModified();
    void onActorModified();
};
```

`buildProjectionParams()`：

```text
1. 读取 vtkVolume::GetMatrix() 得到 M_actor。即使用户不直接拖动 CT actor，系统也允许通过受控接口设置该矩阵。
2. 求 M_actor_inv。
3. 读取 camera position、focal point、viewUp，位于 VTK world。
4. 用 M_actor_inv 转成 patient/physics coordinate。
5. rayDir = normalize(focal - source)。
6. SOD = length(focal - source)。
7. detectorU = normalize(cross(viewUp, rayDir))。
8. detectorV = normalize(cross(detectorU, rayDir))。
9. detectorCenter = source + rayDir * SID。
10. 填充 ProjectionParams。
```

`applyToVtk()`：

```text
1. 读取 M_actor。
2. source patient -> world。
3. isocenter patient -> world。
4. detectorV patient -> world viewUp。
5. 写入 vtkCamera position、focalPoint、viewUp。
6. 使用 blocking flag 防止事件循环。
```

事件链：

```text
camera modified -> buildProjectionParams -> XrayController -> renderAsync
UI angle changed -> applyToVtk -> renderAsync
volume loaded -> default AP params -> applyToVtk -> setVolume -> renderAsync
```

### 9.8 AP/LAT 预设

默认以 CT patient bounds 中心作为 isocenter。

AP：

```text
primaryAngle = 0 deg
secondaryAngle = 0 deg
source roughly posterior/anterior axis，根据显示约定固定
```

LAT：

```text
primaryAngle = 90 deg
secondaryAngle = 0 deg
```

从角度构建 source：

```text
source = isocenter + {
    -SOD * sin(primary) * cos(secondary),
     SOD * sin(secondary),
    -SOD * cos(primary) * cos(secondary)
}
```

具体轴向符号必须通过 phantom 和医学显示约定测试确认。

### 9.9 器械投影叠加

v0.1 器械不体素化进 DRR，采用解析几何 overlay：

```text
1. 对每个可见 instrument 计算 entry 和 endpoint。
2. 将 patient point 投影到 detector plane。
3. 得到 2D line segment。
4. 根据 diameter 和深度近似计算屏幕宽度。
5. 绘制高亮线段和端点。
```

点投影：

```text
ray = point - source
intersect ray with detector plane:
    detectorPlaneNormal = normalize(detectorCenter - source)
    t = dot(detectorCenter - source, normal) / dot(ray, normal)
    hit = source + t * ray
u = dot(hit - detectorCenter, detectorU) / pixelSpacing + width / 2
v = dot(hit - detectorCenter, detectorV) / pixelSpacing + height / 2
```

测试：

- 已知中心线在 AP/LAT 下投影位置正确。
- 改变 length 后投影端点移动正确。
- 改变 diameter 后 overlay 宽度变化正确。

### 9.10 DRR 测试点

- FR-XRAY-001：球体、圆柱和阶梯 phantom 输出正确。
- FR-XRAY-002：AP/LAT 预设 source 和 detector 几何正确。
- FR-XRAY-003：自定义角度修改后 ProjectionParams 更新。
- FR-XRAY-004：器械投影与解析结果一致。
- FR-XRAY-005：512 x 512 实时预览可交互。
- FR-XRAY-006：CPU reference 确定性输出。
- FR-XRAY-007：GPU 与 CPU 误差在阈值内。

## 10. Xray UI 与实时刷新详细设计

### 10.1 XrayController

```cpp
class XrayController : public QObject {
    Q_OBJECT
public:
    void setVolume(const VolumeData& volume);
    void setPlan(const SurgicalPlan& plan);
    void setPreset(XrayPreset preset);
    void setAngles(double primaryDeg, double secondaryDeg);
    void setSid(double sidMm);
    void setResolution(int width, int height);
    void requestRender();

signals:
    void drrImageReady(const DrrImage& image);
    void renderFailed(const ErrorInfo& error);

private:
    void renderLatestAsync();
};
```

### 10.2 版本号防旧帧

```text
1. 每次参数变化时 frameRequestId++。
2. renderAsync 捕获当前 requestId。
3. 回调返回时比较 requestId 是否等于 latestRequestId。
4. 如果不是最新帧，丢弃。
```

### 10.3 刷新策略

- 拖动角度时使用 512 x 512 preview。
- 停止拖动 300 ms 后可触发更高质量渲染。
- 器械只改变 overlay 时可先只刷新 overlay，不必重算 DRR。
- X 射线几何改变时必须重算 DRR。
- volume 改变时必须重传 GPU volume。

## 11. 工程保存详细设计

### 11.1 工程文件格式

v0.1 使用单文件工程包，建议扩展名为 `.mprproj`。工程包内部必须包含 manifest JSON；manifest 是工程数据的主入口。

建议包结构：

```text
project.mprproj
  manifest.json
  thumbnails/
  exports/
  cache/
```

`manifest.json` 示例：

```json
{
  "schemaVersion": "0.1",
  "softwareVersion": "0.1.0",
  "createdAt": "2026-05-03T00:00:00Z",
  "dicom": {
    "sourceFolder": "D:/data/case001",
    "studyUid": "",
    "seriesUid": "",
    "dataHash": ""
  },
  "plan": {
    "instruments": []
  },
  "xrayViews": [],
  "viewState": {}
}
```

v0.1 默认不把原始 DICOM 数据复制进工程包，只保存 DICOM 引用路径、series 标识和数据摘要。若后续需要离线携带数据，应作为独立受控需求处理。

### 11.2 保存逻辑

```text
1. ProjectController 收集 Case。
2. ProjectSerializer 将 domain model 转 manifest JSON。
3. PackageWriter 创建临时工程包。
4. 写入 manifest JSON、缩略图、必要缓存和导出引用。
5. flush 成功后原子替换目标工程包。
6. 返回保存结果。
```

### 11.3 加载逻辑

```text
1. 打开工程包。
2. 读取 manifest JSON。
3. 校验 schemaVersion。
4. 校验 DICOM 引用路径和数据摘要。
5. 重新加载 DICOM volume。
6. 反序列化 instruments、xrayViews、viewState。
7. 校验所有 instrument 参数。
8. 更新 Case。
9. 刷新 MPR/3D/Xray。
```

### 11.4 测试点

- FR-PROJ-001：保存工程成功。
- FR-PROJ-002：加载后数据一致。
- FR-PROJ-003：schemaVersion 和 softwareVersion 存在。
- FR-PROJ-004：导出 DRR PNG。
- FR-PROJ-005：导出 MPR/3D 截图。

## 12. Application Command 详细设计

### 12.1 命令接口

```cpp
class ICommand {
public:
    virtual ~ICommand() = default;
    virtual Result<void> execute() = 0;
    virtual Result<void> undo() = 0;
    virtual std::string name() const = 0;
};
```

初始命令：

- `AddInstrumentCommand`
- `UpdateInstrumentCommand`
- `RemoveInstrumentCommand`
- `SetCrosshairCommand`
- `SetXrayParamsCommand`

### 12.2 Undo/Redo 边界

建议：

- 一次属性面板确认修改形成一个 undo step。
- 拖拽过程中连续更新，鼠标释放时合并为一个 undo step。
- 删除器械必须可撤销。
- DICOM 导入不作为 undo step，属于 case reset。

## 13. 错误处理详细设计

### 13.1 Result 类型

```cpp
template <typename T>
class Result {
public:
    bool ok() const;
    const T& value() const;
    const ErrorInfo& error() const;
};

struct ErrorInfo {
    std::string code;
    std::string message;
    std::string detail;
    bool recoverable = true;
};
```

### 13.2 UI 显示策略

- 用户可修正错误：显示对话框或面板提示。
- 技术诊断：写入日志。
- 数据风险：阻止继续规划。
- 导出失败：保留当前工程状态，不影响规划。

## 14. 测试详细设计

### 14.1 单元测试目标

```text
test_core_coordinates
test_dicom_loader
test_mpr_reslice
test_instrument_model
test_instrument_geometry
test_drr_cpu
test_project_serializer
```

### 14.2 功能测试目标

```text
functional_import_dicom
functional_mpr_sync
functional_create_guide_pin
functional_create_pedicle_screw
functional_edit_instrument
functional_generate_ap_drr
functional_generate_lat_drr
functional_save_load_project
```

### 14.3 算法验证测试目标

```text
validation_drr_sphere_phantom
validation_drr_cylinder_phantom
validation_drr_step_hu_phantom
validation_projection_known_points
validation_cpu_gpu_drr_difference
```

### 14.4 测试数据结构

```text
tests/data/
  dicom/
    ct_phantom_single_series/
  phantom/
    sphere_128/
    cylinder_128/
    step_hu_128/
  expected/
    drr_sphere_ap.json
    drr_cylinder_lat.json
```

医疗影像测试数据不得直接提交真实患者数据。真实数据只能存放在本地受控路径或受控数据管理系统中。

## 15. 详细设计追踪

| 设计章节 | 需求 |
| --- | --- |
| 第 4 章 DICOM 导入 | FR-DICOM-001 到 FR-DICOM-004 |
| 第 5 章 坐标系统 | NFR-ACC-001、FR-INS-004、FR-XRAY-004 |
| 第 6 章 MPR | FR-MPR-001 到 FR-MPR-005 |
| 第 7 章 3D 显示 | FR-3D-001 到 FR-3D-004 |
| 第 8 章 器械规划 | FR-INS-001 到 FR-INS-007 |
| 第 9 章 DRR | FR-XRAY-001 到 FR-XRAY-007 |
| 第 11 章 工程保存 | FR-PROJ-001 到 FR-PROJ-005 |
| 第 14 章 测试 | NFR-TEST-001 到 NFR-TEST-003 |

## 16. 已确认设计决策

| ID | 决策 | 详细设计影响 |
| --- | --- | --- |
| D-DEC-001 | DICOM 库选择 DCMTK。 | `DicomVolumeLoader` 基于 DCMTK 实现 tag 读取、像素装载、序列校验和错误诊断。 |
| D-DEC-002 | GPU DRR 技术路线选择 CUDA。 | `CudaDrrEngine` 使用 CUDA 3D texture、kernel 和 async stream；CPU DRR 保留为 reference。 |
| D-DEC-003 | 允许 CT actor 通过矩阵进行非交互式移动。 | `VtkPhysicsAdapter` 必须每次从 vtkVolume 读取 `M_actor`，用 `M_actor_inv` 将 camera 转换到 patient/physics。 |
| D-DEC-004 | Project JSON 打包成单文件工程包。 | `ProjectSerializer` 负责 manifest JSON，`PackageWriter/PackageReader` 负责包读写和原子替换。 |
| D-DEC-005 | DRR 输出默认不加入水印。 | `XrayViewAdapter` 和导出 metadata 标识仿真属性，不修改 DRR 像素。 |
| D-DEC-006 | MPR 核心采用 `vtkImageReslice + 外部控制器`。 | `MprViewState` 是 MPR 状态来源，`vtkImageReslice` 只作为采样执行器，不使用 `vtkResliceCursorWidget` 承载核心状态。 |

## 17. 待确认设计问题

- AP/LAT 的 patient axis 符号需要用 phantom 和医生习惯共同确认。
- 目标 GPU 最低型号、显存和 CUDA runtime 版本。
- 单文件工程包是否需要加密、签名或完整性校验。
- v0.1 是否需要报告导出。

## 18. 版本记录

| 版本 | 日期 | 说明 |
| --- | --- | --- |
| v0.1 | 2026-05-03 | 建立 DICOM、坐标、MPR、3D、器械、DRR、工程保存和测试框架详细设计基线。 |
