#include "measurement/persistence/ProjectManifest.h"

#include <gtest/gtest.h>

namespace measurement {
namespace {

TEST(ProjectManifestTests, SerializeProjectManifest_IncludesSchemaAndDicomReference)
{
    ProjectManifest manifest;
    manifest.dicomSourceFolder = "D:/data/case001";
    manifest.seriesUid = "series";
    manifest.studyUid = "study";

    const std::string json = serializeProjectManifest(manifest);
    EXPECT_NE(json.find("\"schemaVersion\": \"0.1\""), std::string::npos);
    EXPECT_NE(json.find("D:/data/case001"), std::string::npos);
    EXPECT_NE(json.find("\"seriesUid\": \"series\""), std::string::npos);
}

}  // namespace
}  // namespace measurement
