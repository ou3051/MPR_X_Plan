#pragma once

#include "measurement/core/Instrument.h"
#include "measurement/core/Xray.h"

#include <string>

namespace measurement {

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
