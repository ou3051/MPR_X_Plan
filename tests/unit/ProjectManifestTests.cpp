#include "measurement/persistence/ProjectManifest.h"

#include <gtest/gtest.h>

#include <filesystem>

namespace measurement {
namespace {

[[nodiscard]] Instrument projectInstrument()
{
    Instrument instrument;
    instrument.id = "screw-1";
    instrument.type = InstrumentType::PedicleScrew;
    instrument.entryPointPatientMm = {10.0, 20.0, 30.0};
    instrument.directionPatientUnit = {0.0, 1.0, 0.0};
    instrument.lengthMm = 45.0;
    instrument.diameterMm = 6.5;
    instrument.visible = false;
    instrument.locked = true;
    instrument.label = "Left L4";
    return instrument;
}

[[nodiscard]] ProjectManifest projectManifest()
{
    ProjectManifest manifest;
    manifest.dicomSourceFolder = "D:/data/case001";
    manifest.seriesUid = "series";
    manifest.studyUid = "study";
    manifest.dataHash = "hash";
    manifest.xrayView.preset = XrayPreset::Oblique;
    manifest.xrayView.projection.sourcePosPatientMm = {1.0, 2.0, 3.0};
    manifest.xrayView.projection.detectorCenterPatientMm = {4.0, 5.0, 6.0};
    manifest.xrayView.projection.sidMm = 1100.0;
    manifest.xrayView.projection.sodMm = 750.0;
    manifest.xrayView.windowCenter = 0.25;
    manifest.xrayView.windowWidth = 2.5;
    manifest.mprView.crosshairPatientMm = {7.0, 8.0, 9.0};
    manifest.view3d.cameraPositionPatientMm = {100.0, 200.0, 300.0};
    EXPECT_TRUE(manifest.plan.addInstrument(projectInstrument()).ok());
    return manifest;
}

TEST(ProjectManifestTests, SerializeProjectManifest_IncludesSchemaAndDicomReference)
{
    const ProjectManifest manifest = projectManifest();

    const std::string json = serializeProjectManifest(manifest);
    EXPECT_NE(json.find("\"schemaVersion\": \"0.1\""), std::string::npos);
    EXPECT_NE(json.find("D:/data/case001"), std::string::npos);
    EXPECT_NE(json.find("\"seriesUid\": \"series\""), std::string::npos);
    EXPECT_NE(json.find("\"instruments\""), std::string::npos);
    EXPECT_NE(json.find("\"type\": \"PedicleScrew\""), std::string::npos);
    EXPECT_NE(json.find("\"xrayView\""), std::string::npos);
    EXPECT_NE(json.find("\"mprView\""), std::string::npos);
    EXPECT_NE(json.find("\"view3d\""), std::string::npos);
}

TEST(ProjectManifestTests, DeserializeProjectManifest_RestoresPlanDicomAndXray)
{
    const ProjectManifest manifest = projectManifest();
    const Result<ProjectManifest> restored = deserializeProjectManifest(serializeProjectManifest(manifest));

    ASSERT_TRUE(restored.ok()) << restored.error().message << ": " << restored.error().detail;
    EXPECT_EQ(restored.value().dicomSourceFolder, manifest.dicomSourceFolder);
    EXPECT_EQ(restored.value().studyUid, manifest.studyUid);
    EXPECT_EQ(restored.value().seriesUid, manifest.seriesUid);
    EXPECT_EQ(restored.value().dataHash, manifest.dataHash);
    ASSERT_EQ(restored.value().plan.instruments().size(), 1U);
    const Instrument& instrument = restored.value().plan.instruments()[0];
    EXPECT_EQ(instrument.id, "screw-1");
    EXPECT_EQ(instrument.type, InstrumentType::PedicleScrew);
    EXPECT_FALSE(instrument.visible);
    EXPECT_TRUE(instrument.locked);
    EXPECT_TRUE(nearlyEqual(instrument.entryPointPatientMm, Vec3d{10.0, 20.0, 30.0}));
    EXPECT_TRUE(nearlyEqual(instrument.directionPatientUnit, Vec3d{0.0, 1.0, 0.0}));
    EXPECT_DOUBLE_EQ(restored.value().xrayView.projection.sidMm, 1100.0);
    EXPECT_TRUE(nearlyEqual(restored.value().mprView.crosshairPatientMm, Vec3d{7.0, 8.0, 9.0}));
}

TEST(ProjectManifestTests, ProjectFile_SaveAndLoadRoundTripsManifestEntry)
{
    const ProjectManifest manifest = projectManifest();
    const std::filesystem::path path = std::filesystem::temp_directory_path() / "measurement_project_test.mprproj";
    std::filesystem::remove(path);

    const Result<void> saveResult = saveProjectFile(manifest, path.string());
    ASSERT_TRUE(saveResult.ok()) << saveResult.error().message << ": " << saveResult.error().detail;

    const std::string packageJson = serializeProjectFile(manifest);
    EXPECT_NE(packageJson.find("\"manifest.json\""), std::string::npos);

    const Result<ProjectManifest> loaded = loadProjectFile(path.string());
    ASSERT_TRUE(loaded.ok()) << loaded.error().message << ": " << loaded.error().detail;
    EXPECT_EQ(loaded.value().dicomSourceFolder, "D:/data/case001");
    ASSERT_EQ(loaded.value().plan.instruments().size(), 1U);
    EXPECT_EQ(loaded.value().plan.instruments()[0].label, "Left L4");

    std::filesystem::remove(path);
}

TEST(ProjectManifestTests, DeserializeProjectManifest_RejectsUnsupportedSchema)
{
    std::string json = serializeProjectManifest(projectManifest());
    const size_t versionPos = json.find("\"schemaVersion\": \"0.1\"");
    ASSERT_NE(versionPos, std::string::npos);
    json.replace(versionPos, std::string("\"schemaVersion\": \"0.1\"").size(), "\"schemaVersion\": \"9.9\"");

    const Result<ProjectManifest> restored = deserializeProjectManifest(json);
    ASSERT_FALSE(restored.ok());
    EXPECT_EQ(restored.error().code, "PROJECT_MANIFEST_INVALID");
}

}  // namespace
}  // namespace measurement
