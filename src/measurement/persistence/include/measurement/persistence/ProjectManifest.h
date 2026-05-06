#pragma once

#include "measurement/core/Instrument.h"
#include "measurement/core/Result.h"
#include "measurement/core/Xray.h"

#include <string>
#include <vector>

namespace measurement {

struct MprViewSnapshot {
    Vec3d crosshairPatientMm;
    double zoom = 1.0;
    double windowCenterHu = 400.0;
    double windowWidthHu = 2000.0;
};

struct View3dSnapshot {
    Vec3d cameraPositionPatientMm{0.0, -500.0, 0.0};
    Vec3d cameraFocalPointPatientMm;
    Vec3d cameraUpPatientUnit{0.0, 0.0, 1.0};
    double zoom = 1.0;
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
    std::vector<XrayView> xrayViews;
    MprViewSnapshot mprView;
    View3dSnapshot view3d;
};

[[nodiscard]] std::string serializeProjectManifest(const ProjectManifest& manifest);
[[nodiscard]] Result<ProjectManifest> deserializeProjectManifest(const std::string& json);
[[nodiscard]] std::string serializeProjectFile(const ProjectManifest& manifest);
[[nodiscard]] Result<void> saveProjectFile(const ProjectManifest& manifest, const std::string& path);
[[nodiscard]] Result<ProjectManifest> loadProjectFile(const std::string& path);

}  // namespace measurement
