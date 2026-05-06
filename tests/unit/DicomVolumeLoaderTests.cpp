#include "measurement/dicom/DicomVolumeLoader.h"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <process.h>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#ifndef MEASUREMENT_DCMTK_DUMP2DCM_PATH
#define MEASUREMENT_DCMTK_DUMP2DCM_PATH ""
#endif

namespace measurement {
namespace {

class ScopedTempDir {
public:
    ScopedTempDir()
    {
        const auto uniqueId = std::chrono::steady_clock::now().time_since_epoch().count();
        m_path = std::filesystem::temp_directory_path()
            / ("measurement_dicom_loader_tests_" + std::to_string(uniqueId));
        std::filesystem::create_directories(m_path);
    }

    ~ScopedTempDir()
    {
        std::error_code error;
        std::filesystem::remove_all(m_path, error);
    }

    [[nodiscard]] const std::filesystem::path& path() const { return m_path; }

private:
    std::filesystem::path m_path;
};

TEST(DicomVolumeLoaderTests, LoadFolder_RejectsMissingDirectory)
{
    DicomVolumeLoader loader;
    const auto result = loader.loadFolder("Z:/definitely/not/a/real/dicom/folder");
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, "DICOM_FOLDER_NOT_FOUND");
}

#if MEASUREMENT_HAVE_DCMTK

struct SliceSpec {
    std::string studyUid = "1.2.826.0.1.3680043.10.54321.100";
    std::string seriesUid = "1.2.826.0.1.3680043.10.54321.200";
    std::string sopClassUid = "1.2.840.10008.5.1.4.1.1.2";
    std::string modality = "CT";
    std::string patientPositionCode = "HFS";
    std::array<double, 3> imagePositionPatientMm{10.0, 20.0, 30.0};
    std::array<double, 6> imageOrientationPatient{1.0, 0.0, 0.0, 0.0, 1.0, 0.0};
    std::array<double, 2> pixelSpacingMm{0.8, 0.6};  // DICOM order: row, column
    std::optional<double> sliceThicknessMm{1.5};
    uint16_t rows = 2;
    uint16_t columns = 3;
    uint16_t bitsAllocated = 16;
    uint16_t bitsStored = 16;
    uint16_t pixelRepresentation = 1;
    double rescaleSlope = 2.0;
    double rescaleIntercept = -1000.0;
    std::vector<int16_t> pixels{500, 501, 502, 503, 504, 505};
    bool includeImagePositionPatient = true;
    bool includeImageOrientationPatient = true;
    bool includePatientPosition = true;
    bool includePixelSpacing = true;
    bool includeSliceThickness = true;
    bool includeRescaleSlope = true;
    bool includeRescaleIntercept = true;
    bool includePixelData = true;
};

[[nodiscard]] std::string dump2dcmPath()
{
    return MEASUREMENT_DCMTK_DUMP2DCM_PATH;
}

[[nodiscard]] std::string formatVec3(Vec3d value)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(6)
        << value.x << ", " << value.y << ", " << value.z;
    return out.str();
}

[[nodiscard]] std::optional<std::filesystem::path> realDicomFolderFromEnv()
{
    char* raw = nullptr;
    size_t length = 0;
    const errno_t error = _dupenv_s(&raw, &length, "MEASUREMENT_REAL_DICOM_FOLDER");
    if (error != 0 || raw == nullptr || *raw == '\0') {
        free(raw);
        return std::nullopt;
    }
    const std::filesystem::path folder(raw);
    free(raw);
    return folder;
}

void requireDump2dcm()
{
    if (dump2dcmPath().empty() || !std::filesystem::exists(dump2dcmPath())) {
        GTEST_SKIP() << "dump2dcm.exe is unavailable for DICOM fixture generation.";
    }
}

[[nodiscard]] std::string joinNumbers(const std::vector<double>& values)
{
    std::ostringstream out;
    out << std::setprecision(12);
    for (size_t index = 0; index < values.size(); ++index) {
        if (index > 0) {
            out << "\\";
        }
        out << values[index];
    }
    return out.str();
}

void writeLittleEndianPixels(const std::filesystem::path& path, const std::vector<int16_t>& pixels)
{
    std::ofstream out(path, std::ios::binary);
    ASSERT_TRUE(out.is_open());
    for (int16_t pixel : pixels) {
        const uint16_t raw = static_cast<uint16_t>(pixel);
        const char bytes[2] = {
            static_cast<char>(raw & 0xffU),
            static_cast<char>((raw >> 8U) & 0xffU),
        };
        out.write(bytes, 2);
    }
}

void writeTextFile(const std::filesystem::path& path, const std::string& contents)
{
    std::ofstream out(path, std::ios::binary);
    ASSERT_TRUE(out.is_open());
    out << contents;
}

void writeSliceFile(const std::filesystem::path& path, const SliceSpec& spec, int sopInstanceSuffix)
{
    requireDump2dcm();

    const std::filesystem::path rawPath = path;
    const std::filesystem::path dumpPath = path.parent_path() / (path.stem().string() + ".dump");
    const std::filesystem::path pixelPath = path.parent_path() / (path.stem().string() + ".raw");
    const std::string sopInstanceUid = spec.seriesUid + "." + std::to_string(sopInstanceSuffix);

    std::ostringstream dump;
    dump << "(0008,0016) UI [" << spec.sopClassUid << "]\n";
    dump << "(0008,0018) UI [" << sopInstanceUid << "]\n";
    dump << "(0008,0060) CS [" << spec.modality << "]\n";
    dump << "(0020,000d) UI [" << spec.studyUid << "]\n";
    dump << "(0020,000e) UI [" << spec.seriesUid << "]\n";
    dump << "(0028,0010) US " << spec.rows << "\n";
    dump << "(0028,0011) US " << spec.columns << "\n";
    dump << "(0028,0002) US 1\n";
    dump << "(0028,0004) CS [MONOCHROME2]\n";
    dump << "(0028,0100) US " << spec.bitsAllocated << "\n";
    dump << "(0028,0101) US " << spec.bitsStored << "\n";
    dump << "(0028,0102) US " << (spec.bitsStored - 1U) << "\n";
    dump << "(0028,0103) US " << spec.pixelRepresentation << "\n";

    if (spec.includeImagePositionPatient) {
        dump << "(0020,0032) DS ["
             << joinNumbers({
                    spec.imagePositionPatientMm[0],
                    spec.imagePositionPatientMm[1],
                    spec.imagePositionPatientMm[2],
                })
             << "]\n";
    }
    if (spec.includeImageOrientationPatient) {
        dump << "(0020,0037) DS ["
             << joinNumbers({
                    spec.imageOrientationPatient[0],
                    spec.imageOrientationPatient[1],
                    spec.imageOrientationPatient[2],
                    spec.imageOrientationPatient[3],
                    spec.imageOrientationPatient[4],
                    spec.imageOrientationPatient[5],
                })
             << "]\n";
    }
    if (spec.includePatientPosition) {
        dump << "(0018,5100) CS [" << spec.patientPositionCode << "]\n";
    }
    if (spec.includePixelSpacing) {
        dump << "(0028,0030) DS ["
             << joinNumbers({spec.pixelSpacingMm[0], spec.pixelSpacingMm[1]})
             << "]\n";
    }
    if (spec.includeSliceThickness && spec.sliceThicknessMm.has_value()) {
        dump << "(0018,0050) DS [" << joinNumbers({*spec.sliceThicknessMm}) << "]\n";
    }
    if (spec.includeRescaleSlope) {
        dump << "(0028,1053) DS [" << joinNumbers({spec.rescaleSlope}) << "]\n";
    }
    if (spec.includeRescaleIntercept) {
        dump << "(0028,1052) DS [" << joinNumbers({spec.rescaleIntercept}) << "]\n";
    }
    if (spec.includePixelData) {
        ASSERT_EQ(spec.pixels.size(), static_cast<size_t>(spec.rows) * static_cast<size_t>(spec.columns));
        writeLittleEndianPixels(pixelPath, spec.pixels);
        dump << "(7fe0,0010) OW =" << pixelPath.string() << "\n";
    }

    writeTextFile(dumpPath, dump.str());
    const std::string dump2dcm = dump2dcmPath();
    const std::string dumpPathString = dumpPath.string();
    const std::string rawPathString = rawPath.string();
    const char* const arguments[] = {
        dump2dcm.c_str(),
        "+te",
        dumpPathString.c_str(),
        rawPathString.c_str(),
        nullptr,
    };
    const intptr_t exitCode = _spawnv(_P_WAIT, dump2dcm.c_str(), arguments);
    ASSERT_EQ(exitCode, 0);

    std::error_code cleanupError;
    std::filesystem::remove(dumpPath, cleanupError);
    cleanupError.clear();
    std::filesystem::remove(pixelPath, cleanupError);
}

TEST(DicomVolumeLoaderTests, LoadFolder_ReportsEmptyWhenNoReadableDicomExists)
{
    ScopedTempDir dir;
    std::ofstream(dir.path() / "note.txt") << "not a dicom";

    DicomVolumeLoader loader;
    const auto result = loader.loadFolder(dir.path());

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, "DICOM_EMPTY_FOLDER");
}

TEST(DicomVolumeLoaderTests, LoadFolder_ReportsNoCtSeriesWhenOnlyNonCtDicomExists)
{
    ScopedTempDir dir;
    SliceSpec spec;
    spec.modality = "MR";
    writeSliceFile(dir.path() / "mr_slice.dcm", spec, 1);

    DicomVolumeLoader loader;
    const auto result = loader.loadFolder(dir.path());

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, "DICOM_NO_CT_SERIES");
}

TEST(DicomVolumeLoaderTests, LoadFolder_ReportsMissingTagForInvalidCtSlice)
{
    ScopedTempDir dir;
    SliceSpec spec;
    spec.includeImagePositionPatient = false;
    writeSliceFile(dir.path() / "missing_ipp.dcm", spec, 1);

    DicomVolumeLoader loader;
    const auto result = loader.loadFolder(dir.path());

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, "DICOM_MISSING_TAG");
    EXPECT_NE(result.error().detail.find("ImagePositionPatient"), std::string::npos);
}

TEST(DicomVolumeLoaderTests, LoadFolder_RejectsMultipleCtSeries)
{
    ScopedTempDir dir;
    SliceSpec first;
    SliceSpec second;
    second.seriesUid = "1.2.826.0.1.3680043.10.54321.201";
    second.imagePositionPatientMm[2] += 1.5;

    writeSliceFile(dir.path() / "series_a_1.dcm", first, 1);
    writeSliceFile(dir.path() / "series_b_1.dcm", second, 2);

    DicomVolumeLoader loader;
    const auto result = loader.loadFolder(dir.path());

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, "DICOM_MULTI_SERIES_UNSUPPORTED")
        << result.error().message << " :: " << result.error().detail;
}

TEST(DicomVolumeLoaderTests, LoadFolder_RejectsInconsistentGeometry)
{
    ScopedTempDir dir;
    SliceSpec first;
    SliceSpec second;
    second.imagePositionPatientMm[2] += 2.0;
    second.pixelSpacingMm = {0.8, 0.7};

    writeSliceFile(dir.path() / "slice_1.dcm", first, 1);
    writeSliceFile(dir.path() / "slice_2.dcm", second, 2);

    DicomVolumeLoader loader;
    const auto result = loader.loadFolder(dir.path());

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, "DICOM_INCONSISTENT_GEOMETRY")
        << result.error().message << " :: " << result.error().detail;
}

TEST(DicomVolumeLoaderTests, LoadFolder_BuildsVolumeMetadataHuAndStableHash)
{
    ScopedTempDir dir;
    SliceSpec first;
    SliceSpec second;
    second.imagePositionPatientMm[2] += 1.5;
    second.pixels = {506, 507, 508, 509, 510, 511};
    first.patientPositionCode = "FFP";
    second.patientPositionCode = "FFP";

    writeSliceFile(dir.path() / "slice_1.dcm", first, 1);
    writeSliceFile(dir.path() / "slice_2.dcm", second, 2);

    DicomVolumeLoader loader;
    const auto loadedA = loader.loadFolder(dir.path());
    const auto loadedB = loader.loadFolder(dir.path());

    ASSERT_TRUE(loadedA.ok()) << loadedA.error().code << " :: " << loadedA.error().detail;
    ASSERT_TRUE(loadedB.ok()) << loadedB.error().code << " :: " << loadedB.error().detail;

    const VolumeData& volume = loadedA.value();
    EXPECT_EQ(volume.metadata.dimensions.x, 3);
    EXPECT_EQ(volume.metadata.dimensions.y, 2);
    EXPECT_EQ(volume.metadata.dimensions.z, 2);
    EXPECT_TRUE(nearlyEqual(volume.metadata.spacingMm, Vec3d{0.6, 0.8, 1.5}));
    EXPECT_TRUE(nearlyEqual(volume.metadata.originPatientMm, Vec3d{10.0, 20.0, 30.0}));
    EXPECT_TRUE(nearlyEqual(volume.metadata.rowDirectionPatient, Vec3d{1.0, 0.0, 0.0}));
    EXPECT_TRUE(nearlyEqual(volume.metadata.columnDirectionPatient, Vec3d{0.0, 1.0, 0.0}));
    EXPECT_TRUE(nearlyEqual(volume.metadata.sliceDirectionPatient, Vec3d{0.0, 0.0, 1.0}));
    EXPECT_EQ(volume.metadata.minHu, 0);
    EXPECT_EQ(volume.metadata.maxHu, 22);
    ASSERT_TRUE(volume.image);
    EXPECT_EQ(volume.image->voxelHu(0, 0, 0), 0);
    EXPECT_EQ(volume.image->voxelHu(1, 0, 0), 2);
    EXPECT_EQ(volume.image->voxelHu(2, 1, 1), 22);
    EXPECT_EQ(volume.seriesUid, first.seriesUid);
    EXPECT_EQ(volume.studyUid, first.studyUid);
    EXPECT_EQ(volume.patientPositionCode, "FFP");
    EXPECT_EQ(volume.sourceFolder, dir.path().string());
    EXPECT_FALSE(volume.dataHash.empty());
    EXPECT_EQ(volume.dataHash, loadedB.value().dataHash);

    ScopedTempDir changedDir;
    writeSliceFile(changedDir.path() / "slice_1.dcm", first, 1);
    second.pixels[5] += 1;
    writeSliceFile(changedDir.path() / "slice_2.dcm", second, 2);

    const auto changed = loader.loadFolder(changedDir.path());
    ASSERT_TRUE(changed.ok()) << changed.error().code << " :: " << changed.error().detail;
    EXPECT_NE(volume.dataHash, changed.value().dataHash);
}

TEST(DicomVolumeLoaderTests, LoadFolder_CompatibleModeKeepsLongestContinuousSliceRun)
{
    ScopedTempDir dir;
    SliceSpec first;
    SliceSpec second = first;
    SliceSpec third = first;
    SliceSpec outlier = first;
    second.imagePositionPatientMm[2] += 1.5;
    third.imagePositionPatientMm[2] += 3.0;
    outlier.imagePositionPatientMm[2] += 9.0;
    first.pixels = {500, 501, 502, 503, 504, 505};
    second.pixels = {506, 507, 508, 509, 510, 511};
    third.pixels = {512, 513, 514, 515, 516, 517};
    outlier.pixels = {800, 801, 802, 803, 804, 805};

    writeSliceFile(dir.path() / "slice_1.dcm", first, 1);
    writeSliceFile(dir.path() / "slice_2.dcm", second, 2);
    writeSliceFile(dir.path() / "slice_3.dcm", third, 3);
    writeSliceFile(dir.path() / "slice_4_gap.dcm", outlier, 4);

    DicomVolumeLoader loader;
    const auto loaded = loader.loadFolder(dir.path());

    ASSERT_TRUE(loaded.ok()) << loaded.error().code << " :: " << loaded.error().detail;
    const VolumeData& volume = loaded.value();
    EXPECT_EQ(volume.metadata.dimensions.z, 3);
    EXPECT_TRUE(nearlyEqual(volume.metadata.spacingMm, Vec3d{0.6, 0.8, 1.5}));
    ASSERT_TRUE(volume.image);
    EXPECT_EQ(volume.image->voxelHu(0, 0, 0), 0);
    EXPECT_EQ(volume.image->voxelHu(0, 0, 1), 12);
    EXPECT_EQ(volume.image->voxelHu(0, 0, 2), 24);
}

TEST(DicomVolumeLoaderTests, LoadFolder_DefaultsPatientPositionToHfsWhenMissing)
{
    ScopedTempDir dir;
    SliceSpec first;
    SliceSpec second;
    first.includePatientPosition = false;
    second.includePatientPosition = false;
    second.imagePositionPatientMm[2] += 1.5;

    writeSliceFile(dir.path() / "slice_1.dcm", first, 1);
    writeSliceFile(dir.path() / "slice_2.dcm", second, 2);

    DicomVolumeLoader loader;
    const auto loaded = loader.loadFolder(dir.path());
    ASSERT_TRUE(loaded.ok()) << loaded.error().code << " :: " << loaded.error().detail;
    EXPECT_EQ(loaded.value().patientPositionCode, "HFS");
}

TEST(DicomVolumeLoaderTests, LoadFolder_ReportsRealDicomBaselineWhenRequested)
{
    const auto realFolder = realDicomFolderFromEnv();
    if (!realFolder.has_value()) {
        GTEST_SKIP() << "MEASUREMENT_REAL_DICOM_FOLDER is not set.";
    }
    if (!std::filesystem::exists(*realFolder) || !std::filesystem::is_directory(*realFolder)) {
        GTEST_SKIP() << "MEASUREMENT_REAL_DICOM_FOLDER does not point to an existing directory.";
    }

    DicomVolumeLoader loader;
    const auto loaded = loader.loadFolder(*realFolder);
    ASSERT_TRUE(loaded.ok()) << loaded.error().code << " :: " << loaded.error().detail;

    const VolumeData& volume = loaded.value();
    ASSERT_TRUE(volume.image);

    std::cout
        << "\nREAL_DICOM_BASELINE\n"
        << "folder: " << realFolder->string() << "\n"
        << "dimensions: "
        << volume.metadata.dimensions.x << " x "
        << volume.metadata.dimensions.y << " x "
        << volume.metadata.dimensions.z << "\n"
        << "spacing_mm: " << formatVec3(volume.metadata.spacingMm) << "\n"
        << "origin_patient_mm: " << formatVec3(volume.metadata.originPatientMm) << "\n"
        << "row_direction_patient: " << formatVec3(volume.metadata.rowDirectionPatient) << "\n"
        << "column_direction_patient: " << formatVec3(volume.metadata.columnDirectionPatient) << "\n"
        << "slice_direction_patient: " << formatVec3(volume.metadata.sliceDirectionPatient) << "\n"
        << "min_hu: " << volume.metadata.minHu << "\n"
        << "max_hu: " << volume.metadata.maxHu << "\n"
        << "study_uid: " << volume.studyUid << "\n"
        << "series_uid: " << volume.seriesUid << "\n"
        << "data_hash: " << volume.dataHash << "\n";
}

#else

TEST(DicomVolumeLoaderTests, LoadFolder_ReportsDependencyMissingWhenDcmtkIsUnavailable)
{
    ScopedTempDir dir;

    DicomVolumeLoader loader;
    const auto result = loader.loadFolder(dir.path());

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, "DICOM_DEPENDENCY_MISSING");
}

#endif

}  // namespace
}  // namespace measurement
