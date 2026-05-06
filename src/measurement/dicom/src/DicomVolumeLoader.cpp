#include "measurement/dicom/DicomVolumeLoader.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <cctype>
#include <system_error>
#include <utility>
#include <vector>

#if MEASUREMENT_HAVE_DCMTK
#include <dcmtk/dcmdata/dctk.h>
#include <dcmtk/dcmdata/dcuid.h>
#endif

namespace measurement {

#if MEASUREMENT_HAVE_DCMTK
namespace {

constexpr double kDirectionTolerance = 1.0e-5;
constexpr double kSpacingTolerance = 1.0e-4;
constexpr double kProjectionTolerance = 1.0e-4;
constexpr double kCompatibleSpacingToleranceMm = 0.05;

class DenseHuVolume final : public IImageVolume {
public:
    DenseHuVolume(Size3i dimensions, std::vector<int16_t> voxels)
        : m_dimensions(dimensions)
        , m_voxels(std::move(voxels))
    {
    }

    [[nodiscard]] Size3i dimensions() const override { return m_dimensions; }

    [[nodiscard]] int16_t voxelHu(int i, int j, int k) const override
    {
        const size_t sliceSize = static_cast<size_t>(m_dimensions.x) * static_cast<size_t>(m_dimensions.y);
        const size_t index = static_cast<size_t>(k) * sliceSize
            + static_cast<size_t>(j) * static_cast<size_t>(m_dimensions.x)
            + static_cast<size_t>(i);
        return m_voxels.at(index);
    }

private:
    Size3i m_dimensions{};
    std::vector<int16_t> m_voxels;
};

struct DicomSliceInfo {
    std::filesystem::path path;
    std::string studyUid;
    std::string seriesUid;
    std::string sopClassUid;
    std::string patientPositionCode = "HFS";
    Vec3d imagePositionPatientMm;
    Vec3d rowDirectionPatient;
    Vec3d columnDirectionPatient;
    double rowSpacingMm = 0.0;
    double columnSpacingMm = 0.0;
    std::optional<double> sliceThicknessMm;
    uint16_t rows = 0;
    uint16_t columns = 0;
    uint16_t bitsAllocated = 0;
    uint16_t bitsStored = 0;
    uint16_t pixelRepresentation = 0;
    double rescaleSlope = 1.0;
    double rescaleIntercept = 0.0;
    double sliceProjectionMm = 0.0;
    std::vector<int32_t> rawPixels;
};

[[nodiscard]] ErrorInfo makeError(
    std::string code,
    std::string message,
    std::string detail,
    bool recoverable = true)
{
    return {std::move(code), std::move(message), std::move(detail), recoverable};
}

[[nodiscard]] Result<void> voidFailure(
    std::string code,
    std::string message,
    std::string detail,
    bool recoverable = true)
{
    return Result<void>::failure(makeError(std::move(code), std::move(message), std::move(detail), recoverable));
}

template <typename T>
[[nodiscard]] Result<T> missingTagResult(const std::filesystem::path& path, const std::string& tagName)
{
    return Result<T>::failure(makeError(
        "DICOM_MISSING_TAG",
        "A required DICOM tag is missing.",
        path.string() + " :: " + tagName,
        true));
}

template <typename T>
[[nodiscard]] Result<T> imageBuildFailure(const std::filesystem::path& path, const std::string& detail)
{
    return Result<T>::failure(makeError(
        "DICOM_IMAGE_BUILD_FAILED",
        "Failed to build voxel data from DICOM pixel data.",
        path.string() + " :: " + detail,
        false));
}

[[nodiscard]] bool nearlyEqualDirection(Vec3d lhs, Vec3d rhs)
{
    return nearlyEqual(normalize(lhs), normalize(rhs), kDirectionTolerance);
}

[[nodiscard]] std::string formatDouble(double value)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(6) << value;
    return out.str();
}

[[nodiscard]] Result<std::string> readRequiredString(
    DcmDataset& dataset,
    const DcmTagKey& tag,
    const std::filesystem::path& path,
    const std::string& tagName)
{
    OFString value;
    if (dataset.findAndGetOFStringArray(tag, value).bad() || value.empty()) {
        return missingTagResult<std::string>(path, tagName);
    }
    return Result<std::string>::success(std::string(value.c_str()));
}

[[nodiscard]] std::optional<std::string> readOptionalString(
    DcmDataset& dataset,
    const DcmTagKey& tag)
{
    OFString value;
    if (dataset.findAndGetOFStringArray(tag, value).bad() || value.empty()) {
        return std::nullopt;
    }
    return std::string(value.c_str());
}

[[nodiscard]] std::string normalizedPatientPositionCode(std::optional<std::string> rawCode)
{
    if (!rawCode.has_value()) {
        return "HFS";
    }

    std::string code = std::move(*rawCode);
    std::transform(code.begin(), code.end(), code.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    if (code == "HFS" || code == "HFP" || code == "FFS" || code == "FFP") {
        return code;
    }
    return "HFS";
}

[[nodiscard]] Result<uint16_t> readRequiredUint16(
    DcmDataset& dataset,
    const DcmTagKey& tag,
    const std::filesystem::path& path,
    const std::string& tagName)
{
    Uint16 value = 0;
    if (dataset.findAndGetUint16(tag, value).bad()) {
        return missingTagResult<uint16_t>(path, tagName);
    }
    return Result<uint16_t>::success(static_cast<uint16_t>(value));
}

[[nodiscard]] Result<double> readRequiredFloat64(
    DcmDataset& dataset,
    const DcmTagKey& tag,
    const std::filesystem::path& path,
    const std::string& tagName)
{
    Float64 value = 0.0;
    if (dataset.findAndGetFloat64(tag, value).bad()) {
        return missingTagResult<double>(path, tagName);
    }
    return Result<double>::success(static_cast<double>(value));
}

[[nodiscard]] Result<std::array<double, 2>> readRequiredFloat64Pair(
    DcmDataset& dataset,
    const DcmTagKey& tag,
    const std::filesystem::path& path,
    const std::string& tagName)
{
    std::array<double, 2> values{};
    for (unsigned long index = 0; index < values.size(); ++index) {
        Float64 value = 0.0;
        if (dataset.findAndGetFloat64(tag, value, index).bad()) {
            return missingTagResult<std::array<double, 2>>(path, tagName);
        }
        values[index] = static_cast<double>(value);
    }
    return Result<std::array<double, 2>>::success(values);
}

[[nodiscard]] Result<std::array<double, 3>> readRequiredFloat64Triple(
    DcmDataset& dataset,
    const DcmTagKey& tag,
    const std::filesystem::path& path,
    const std::string& tagName)
{
    std::array<double, 3> values{};
    for (unsigned long index = 0; index < values.size(); ++index) {
        Float64 value = 0.0;
        if (dataset.findAndGetFloat64(tag, value, index).bad()) {
            return missingTagResult<std::array<double, 3>>(path, tagName);
        }
        values[index] = static_cast<double>(value);
    }
    return Result<std::array<double, 3>>::success(values);
}

[[nodiscard]] Result<std::array<double, 6>> readRequiredFloat64Six(
    DcmDataset& dataset,
    const DcmTagKey& tag,
    const std::filesystem::path& path,
    const std::string& tagName)
{
    std::array<double, 6> values{};
    for (unsigned long index = 0; index < values.size(); ++index) {
        Float64 value = 0.0;
        if (dataset.findAndGetFloat64(tag, value, index).bad()) {
            return missingTagResult<std::array<double, 6>>(path, tagName);
        }
        values[index] = static_cast<double>(value);
    }
    return Result<std::array<double, 6>>::success(values);
}

[[nodiscard]] int32_t normalizeStoredSample(int32_t rawValue, uint16_t bitsStored, bool signedPixel)
{
    if (bitsStored == 0 || bitsStored >= 32) {
        return rawValue;
    }

    const uint32_t rawBits = static_cast<uint32_t>(rawValue);
    const uint32_t mask = (static_cast<uint32_t>(1) << bitsStored) - 1U;
    const uint32_t maskedValue = rawBits & mask;
    if (!signedPixel) {
        return static_cast<int32_t>(maskedValue);
    }

    const uint32_t signBit = static_cast<uint32_t>(1) << (bitsStored - 1U);
    if ((maskedValue & signBit) == 0U) {
        return static_cast<int32_t>(maskedValue);
    }
    return static_cast<int32_t>(maskedValue | ~mask);
}

[[nodiscard]] Result<std::vector<int32_t>> readPixelSamples(
    DcmDataset& dataset,
    const std::filesystem::path& path,
    uint16_t rows,
    uint16_t columns,
    uint16_t bitsAllocated,
    uint16_t bitsStored,
    uint16_t pixelRepresentation)
{
    if (!dataset.tagExistsWithValue(DCM_PixelData)) {
        return missingTagResult<std::vector<int32_t>>(path, "PixelData");
    }
    if (bitsAllocated != 8U && bitsAllocated != 16U) {
        return imageBuildFailure<std::vector<int32_t>>(path, "Unsupported BitsAllocated=" + std::to_string(bitsAllocated));
    }
    if (bitsStored == 0U || bitsStored > bitsAllocated) {
        return imageBuildFailure<std::vector<int32_t>>(path, "Unsupported BitsStored=" + std::to_string(bitsStored));
    }
    if (pixelRepresentation > 1U) {
        return imageBuildFailure<std::vector<int32_t>>(path, "Unsupported PixelRepresentation=" + std::to_string(pixelRepresentation));
    }

    const size_t expectedCount = static_cast<size_t>(rows) * static_cast<size_t>(columns);
    std::vector<int32_t> rawPixels(expectedCount, 0);
    const bool signedPixel = pixelRepresentation == 1U;

    if (bitsAllocated == 8U) {
        const Uint8* values = nullptr;
        unsigned long count = 0;
        if (dataset.findAndGetUint8Array(DCM_PixelData, values, &count).bad() || values == nullptr || count < expectedCount) {
            return imageBuildFailure<std::vector<int32_t>>(path, "PixelData byte array is unavailable or truncated.");
        }

        for (size_t index = 0; index < expectedCount; ++index) {
            const int32_t sample = signedPixel
                ? static_cast<int32_t>(static_cast<int8_t>(values[index]))
                : static_cast<int32_t>(values[index]);
            rawPixels[index] = normalizeStoredSample(sample, bitsStored, signedPixel);
        }
        return Result<std::vector<int32_t>>::success(std::move(rawPixels));
    }

    const Uint16* values = nullptr;
    unsigned long count = 0;
    if (dataset.findAndGetUint16Array(DCM_PixelData, values, &count).bad() || values == nullptr || count < expectedCount) {
        return imageBuildFailure<std::vector<int32_t>>(path, "PixelData word array is unavailable or truncated.");
    }
    for (size_t index = 0; index < expectedCount; ++index) {
        rawPixels[index] = normalizeStoredSample(static_cast<int32_t>(values[index]), bitsStored, signedPixel);
    }
    return Result<std::vector<int32_t>>::success(std::move(rawPixels));
}

[[nodiscard]] Result<DicomSliceInfo> readSliceInfo(const std::filesystem::path& path)
{
    DcmFileFormat fileFormat;
    if (fileFormat.loadFile(path.string().c_str()).bad()) {
        return Result<DicomSliceInfo>::failure(makeError("DICOM_NOT_READABLE", "", "", true));
    }

    DcmDataset* dataset = fileFormat.getDataset();
    const auto modality = readRequiredString(*dataset, DCM_Modality, path, "Modality");
    if (!modality.ok()) {
        return Result<DicomSliceInfo>::failure(modality.error());
    }
    if (modality.value() != "CT") {
        return Result<DicomSliceInfo>::failure(makeError("DICOM_NOT_CT", "", "", true));
    }

    const auto sopClassUid = readRequiredString(*dataset, DCM_SOPClassUID, path, "SOPClassUID");
    if (!sopClassUid.ok()) {
        return Result<DicomSliceInfo>::failure(sopClassUid.error());
    }
    if (sopClassUid.value() != UID_CTImageStorage) {
        return Result<DicomSliceInfo>::failure(makeError("DICOM_NOT_CT", "", "", true));
    }

    const auto studyUid = readRequiredString(*dataset, DCM_StudyInstanceUID, path, "StudyInstanceUID");
    if (!studyUid.ok()) {
        return Result<DicomSliceInfo>::failure(studyUid.error());
    }
    const auto seriesUid = readRequiredString(*dataset, DCM_SeriesInstanceUID, path, "SeriesInstanceUID");
    if (!seriesUid.ok()) {
        return Result<DicomSliceInfo>::failure(seriesUid.error());
    }
    const auto imagePosition = readRequiredFloat64Triple(*dataset, DCM_ImagePositionPatient, path, "ImagePositionPatient");
    if (!imagePosition.ok()) {
        return Result<DicomSliceInfo>::failure(imagePosition.error());
    }
    const auto imageOrientation = readRequiredFloat64Six(*dataset, DCM_ImageOrientationPatient, path, "ImageOrientationPatient");
    if (!imageOrientation.ok()) {
        return Result<DicomSliceInfo>::failure(imageOrientation.error());
    }
    const auto pixelSpacing = readRequiredFloat64Pair(*dataset, DCM_PixelSpacing, path, "PixelSpacing");
    if (!pixelSpacing.ok()) {
        return Result<DicomSliceInfo>::failure(pixelSpacing.error());
    }
    const auto rows = readRequiredUint16(*dataset, DCM_Rows, path, "Rows");
    if (!rows.ok()) {
        return Result<DicomSliceInfo>::failure(rows.error());
    }
    const auto columns = readRequiredUint16(*dataset, DCM_Columns, path, "Columns");
    if (!columns.ok()) {
        return Result<DicomSliceInfo>::failure(columns.error());
    }
    const auto bitsAllocated = readRequiredUint16(*dataset, DCM_BitsAllocated, path, "BitsAllocated");
    if (!bitsAllocated.ok()) {
        return Result<DicomSliceInfo>::failure(bitsAllocated.error());
    }
    const auto bitsStored = readRequiredUint16(*dataset, DCM_BitsStored, path, "BitsStored");
    if (!bitsStored.ok()) {
        return Result<DicomSliceInfo>::failure(bitsStored.error());
    }
    const auto pixelRepresentation = readRequiredUint16(*dataset, DCM_PixelRepresentation, path, "PixelRepresentation");
    if (!pixelRepresentation.ok()) {
        return Result<DicomSliceInfo>::failure(pixelRepresentation.error());
    }
    const auto rescaleSlope = readRequiredFloat64(*dataset, DCM_RescaleSlope, path, "RescaleSlope");
    if (!rescaleSlope.ok()) {
        return Result<DicomSliceInfo>::failure(rescaleSlope.error());
    }
    const auto rescaleIntercept = readRequiredFloat64(*dataset, DCM_RescaleIntercept, path, "RescaleIntercept");
    if (!rescaleIntercept.ok()) {
        return Result<DicomSliceInfo>::failure(rescaleIntercept.error());
    }

    std::optional<double> sliceThicknessMm;
    Float64 thicknessValue = 0.0;
    if (dataset->findAndGetFloat64(DCM_SliceThickness, thicknessValue).good()) {
        sliceThicknessMm = static_cast<double>(thicknessValue);
    }

    const auto rawPixels = readPixelSamples(
        *dataset,
        path,
        rows.value(),
        columns.value(),
        bitsAllocated.value(),
        bitsStored.value(),
        pixelRepresentation.value());
    if (!rawPixels.ok()) {
        return Result<DicomSliceInfo>::failure(rawPixels.error());
    }

    DicomSliceInfo slice;
    slice.path = path;
    slice.studyUid = studyUid.value();
    slice.seriesUid = seriesUid.value();
    slice.sopClassUid = sopClassUid.value();
    slice.patientPositionCode = normalizedPatientPositionCode(
        readOptionalString(*dataset, DCM_PatientPosition));
    slice.imagePositionPatientMm = {imagePosition.value()[0], imagePosition.value()[1], imagePosition.value()[2]};
    slice.rowDirectionPatient = {imageOrientation.value()[0], imageOrientation.value()[1], imageOrientation.value()[2]};
    slice.columnDirectionPatient = {imageOrientation.value()[3], imageOrientation.value()[4], imageOrientation.value()[5]};
    slice.rowSpacingMm = pixelSpacing.value()[1];
    slice.columnSpacingMm = pixelSpacing.value()[0];
    slice.sliceThicknessMm = sliceThicknessMm;
    slice.rows = rows.value();
    slice.columns = columns.value();
    slice.bitsAllocated = bitsAllocated.value();
    slice.bitsStored = bitsStored.value();
    slice.pixelRepresentation = pixelRepresentation.value();
    slice.rescaleSlope = rescaleSlope.value();
    slice.rescaleIntercept = rescaleIntercept.value();
    slice.rawPixels = rawPixels.value();

    if (slice.rows == 0U || slice.columns == 0U || slice.rowSpacingMm <= 0.0 || slice.columnSpacingMm <= 0.0) {
        return Result<DicomSliceInfo>::failure(makeError(
            "DICOM_INCONSISTENT_GEOMETRY",
            "DICOM slice geometry is invalid.",
            path.string(),
            true));
    }
    if (length(slice.rowDirectionPatient) == 0.0 || length(slice.columnDirectionPatient) == 0.0) {
        return Result<DicomSliceInfo>::failure(makeError(
            "DICOM_INCONSISTENT_GEOMETRY",
            "DICOM orientation vectors are invalid.",
            path.string(),
            true));
    }

    return Result<DicomSliceInfo>::success(std::move(slice));
}

[[nodiscard]] Result<std::vector<DicomSliceInfo>> scanCtSlices(const std::filesystem::path& folder)
{
    std::vector<DicomSliceInfo> slices;
    bool sawReadableDicom = false;

    std::error_code iterationError;
    for (std::filesystem::directory_iterator it(folder, iterationError), end; it != end; it.increment(iterationError)) {
        if (iterationError) {
            return Result<std::vector<DicomSliceInfo>>::failure(makeError(
                "DICOM_IMAGE_BUILD_FAILED",
                "Failed while scanning the DICOM folder.",
                iterationError.message(),
                false));
        }
        if (!it->is_regular_file()) {
            continue;
        }

        const auto sliceResult = readSliceInfo(it->path());
        if (!sliceResult.ok()) {
            const std::string& code = sliceResult.error().code;
            if (code == "DICOM_NOT_READABLE") {
                continue;
            }
            if (code == "DICOM_NOT_CT") {
                sawReadableDicom = true;
                continue;
            }
            return Result<std::vector<DicomSliceInfo>>::failure(sliceResult.error());
        }

        sawReadableDicom = true;
        slices.push_back(sliceResult.value());
    }

    if (slices.empty()) {
        return Result<std::vector<DicomSliceInfo>>::failure(makeError(
            sawReadableDicom ? "DICOM_NO_CT_SERIES" : "DICOM_EMPTY_FOLDER",
            sawReadableDicom ? "No CT DICOM slices were found in the folder." : "No readable DICOM files were found in the folder.",
            folder.string(),
            true));
    }

    return Result<std::vector<DicomSliceInfo>>::success(std::move(slices));
}

[[nodiscard]] std::optional<double> modalSliceSpacingMm(const std::vector<DicomSliceInfo>& slices);
[[nodiscard]] bool keepLongestCompatibleSliceRun(std::vector<DicomSliceInfo>& slices, double spacingMm);

[[nodiscard]] Result<void> validateAndSortSlices(std::vector<DicomSliceInfo>& slices)
{
    const std::string seriesUid = slices.front().seriesUid;
    const std::string studyUid = slices.front().studyUid;
    const uint16_t rows = slices.front().rows;
    const uint16_t columns = slices.front().columns;
    const uint16_t bitsAllocated = slices.front().bitsAllocated;
    const uint16_t bitsStored = slices.front().bitsStored;
    const uint16_t pixelRepresentation = slices.front().pixelRepresentation;
    const std::string patientPositionCode = slices.front().patientPositionCode;
    const double rowSpacingMm = slices.front().rowSpacingMm;
    const double columnSpacingMm = slices.front().columnSpacingMm;
    const double rescaleSlope = slices.front().rescaleSlope;
    const double rescaleIntercept = slices.front().rescaleIntercept;
    const Vec3d rowDirection = normalize(slices.front().rowDirectionPatient);
    const Vec3d columnDirection = normalize(slices.front().columnDirectionPatient);
    const Vec3d sliceDirection = normalize(cross(rowDirection, columnDirection));

    if (length(sliceDirection) == 0.0) {
        return voidFailure(
            "DICOM_INCONSISTENT_GEOMETRY",
            "DICOM slice orientation is degenerate.",
            slices.front().path.string(),
            true);
    }

    for (DicomSliceInfo& slice : slices) {
        if (slice.seriesUid != seriesUid) {
            return voidFailure(
                "DICOM_MULTI_SERIES_UNSUPPORTED",
                "Multiple CT series are not supported in v0.1.",
                slice.path.string(),
                true);
        }
        if (slice.studyUid != studyUid
            || slice.rows != rows
            || slice.columns != columns
            || slice.bitsAllocated != bitsAllocated
            || slice.bitsStored != bitsStored
            || slice.pixelRepresentation != pixelRepresentation
            || slice.patientPositionCode != patientPositionCode
            || !nearlyEqual(slice.rowSpacingMm, rowSpacingMm, kSpacingTolerance)
            || !nearlyEqual(slice.columnSpacingMm, columnSpacingMm, kSpacingTolerance)
            || !nearlyEqual(slice.rescaleSlope, rescaleSlope, kSpacingTolerance)
            || !nearlyEqual(slice.rescaleIntercept, rescaleIntercept, kSpacingTolerance)
            || !nearlyEqualDirection(slice.rowDirectionPatient, rowDirection)
            || !nearlyEqualDirection(slice.columnDirectionPatient, columnDirection)
            || !nearlyEqualDirection(cross(slice.rowDirectionPatient, slice.columnDirectionPatient), sliceDirection)) {
            return voidFailure(
                "DICOM_INCONSISTENT_GEOMETRY",
                "CT slices in the folder do not share a consistent geometry.",
                slice.path.string(),
                true);
        }
        slice.sliceProjectionMm = dot(slice.imagePositionPatientMm, sliceDirection);
    }

    std::sort(slices.begin(), slices.end(), [](const DicomSliceInfo& lhs, const DicomSliceInfo& rhs) {
        return lhs.sliceProjectionMm < rhs.sliceProjectionMm;
    });

    for (size_t index = 1; index < slices.size(); ++index) {
        const double delta = slices[index].sliceProjectionMm - slices[index - 1].sliceProjectionMm;
        if (delta <= kProjectionTolerance) {
            return voidFailure(
                "DICOM_INCONSISTENT_GEOMETRY",
                "CT slices overlap or are not strictly ordered along the slice axis.",
                slices[index].path.string(),
                true);
        }
    }

    if (slices.size() == 1U) {
        if (!slices.front().sliceThicknessMm.has_value()) {
            return voidFailure(
                "DICOM_MISSING_TAG",
                "A required DICOM tag is missing.",
                slices.front().path.string() + " :: SliceThickness",
                true);
        }
        if (*slices.front().sliceThicknessMm <= 0.0) {
            return voidFailure(
                "DICOM_INCONSISTENT_GEOMETRY",
                "SliceThickness must be positive for a single-slice volume.",
                slices.front().path.string(),
                true);
        }
        return Result<void>::success();
    }

    const double expectedSpacing = (slices.back().sliceProjectionMm - slices.front().sliceProjectionMm)
        / static_cast<double>(slices.size() - 1U);
    if (expectedSpacing <= kProjectionTolerance) {
        return voidFailure(
            "DICOM_INCONSISTENT_GEOMETRY",
            "Slice spacing must be positive.",
            slices.front().path.string(),
            true);
    }

    bool spacingConsistent = true;
    for (size_t index = 1; index < slices.size(); ++index) {
        const double delta = slices[index].sliceProjectionMm - slices[index - 1].sliceProjectionMm;
        if (!nearlyEqual(delta, expectedSpacing, kSpacingTolerance)) {
            spacingConsistent = false;
            break;
        }
    }

    if (!spacingConsistent) {
        const auto compatibleSpacing = modalSliceSpacingMm(slices);
        if (compatibleSpacing.has_value() && keepLongestCompatibleSliceRun(slices, *compatibleSpacing)) {
            return validateAndSortSlices(slices);
        }

        for (size_t index = 1; index < slices.size(); ++index) {
            const double delta = slices[index].sliceProjectionMm - slices[index - 1].sliceProjectionMm;
            if (!nearlyEqual(delta, expectedSpacing, kSpacingTolerance)) {
                return voidFailure(
                    "DICOM_INCONSISTENT_GEOMETRY",
                    "Slice spacing is inconsistent across the CT stack.",
                    slices[index].path.string() + " :: delta=" + formatDouble(delta),
                    true);
            }
        }
    }

    return Result<void>::success();
}

[[nodiscard]] std::optional<double> modalSliceSpacingMm(const std::vector<DicomSliceInfo>& slices)
{
    if (slices.size() < 2U) {
        return std::nullopt;
    }

    std::vector<double> deltas;
    deltas.reserve(slices.size() - 1U);
    for (size_t index = 1; index < slices.size(); ++index) {
        const double delta = slices[index].sliceProjectionMm - slices[index - 1].sliceProjectionMm;
        if (delta > kProjectionTolerance) {
            deltas.push_back(delta);
        }
    }
    if (deltas.empty()) {
        return std::nullopt;
    }
    std::sort(deltas.begin(), deltas.end());

    double bestSpacing = deltas.front();
    size_t bestCount = 0;
    size_t index = 0;
    while (index < deltas.size()) {
        const double seed = deltas[index];
        double sum = 0.0;
        size_t count = 0;
        while (index < deltas.size() && std::abs(deltas[index] - seed) <= kCompatibleSpacingToleranceMm) {
            sum += deltas[index];
            ++count;
            ++index;
        }
        if (count > bestCount) {
            bestCount = count;
            bestSpacing = sum / static_cast<double>(count);
        }
    }
    return bestSpacing;
}

[[nodiscard]] bool keepLongestCompatibleSliceRun(std::vector<DicomSliceInfo>& slices, double spacingMm)
{
    if (slices.size() < 3U || spacingMm <= kProjectionTolerance) {
        return false;
    }

    size_t bestStart = 0;
    size_t bestCount = 1;
    size_t currentStart = 0;
    size_t currentCount = 1;
    for (size_t index = 1; index < slices.size(); ++index) {
        const double delta = slices[index].sliceProjectionMm - slices[index - 1].sliceProjectionMm;
        if (std::abs(delta - spacingMm) <= kCompatibleSpacingToleranceMm) {
            ++currentCount;
        } else {
            if (currentCount > bestCount) {
                bestStart = currentStart;
                bestCount = currentCount;
            }
            currentStart = index;
            currentCount = 1;
        }
    }
    if (currentCount > bestCount) {
        bestStart = currentStart;
        bestCount = currentCount;
    }

    if (bestCount < 2U || bestCount == slices.size()) {
        return false;
    }

    // Compatibility mode: retain the longest physically continuous stack and
    // ignore slices separated by large gaps. This keeps a single CT series
    // usable without inventing missing anatomy or resampling voxel data.
    slices.erase(slices.begin() + static_cast<std::ptrdiff_t>(bestStart + bestCount), slices.end());
    slices.erase(slices.begin(), slices.begin() + static_cast<std::ptrdiff_t>(bestStart));
    return true;
}

[[nodiscard]] double computeSliceSpacingMm(const std::vector<DicomSliceInfo>& slices)
{
    if (slices.size() == 1U) {
        return slices.front().sliceThicknessMm.value_or(1.0);
    }
    return (slices.back().sliceProjectionMm - slices.front().sliceProjectionMm)
        / static_cast<double>(slices.size() - 1U);
}

[[nodiscard]] int16_t clampHu(double hu)
{
    constexpr int16_t kMinHuValue = (std::numeric_limits<int16_t>::lowest)();
    constexpr int16_t kMaxHuValue = (std::numeric_limits<int16_t>::max)();
    const double rounded = std::round(hu);
    if (rounded < static_cast<double>(kMinHuValue)) {
        return kMinHuValue;
    }
    if (rounded > static_cast<double>(kMaxHuValue)) {
        return kMaxHuValue;
    }
    return static_cast<int16_t>(rounded);
}

[[nodiscard]] Result<std::vector<int16_t>> buildHuVolume(const std::vector<DicomSliceInfo>& slices)
{
    const size_t sliceSize = static_cast<size_t>(slices.front().columns) * static_cast<size_t>(slices.front().rows);
    std::vector<int16_t> voxels;
    voxels.reserve(sliceSize * slices.size());

    for (const DicomSliceInfo& slice : slices) {
        if (slice.rawPixels.size() != sliceSize) {
            return imageBuildFailure<std::vector<int16_t>>(slice.path, "PixelData size does not match Rows * Columns.");
        }
        for (int32_t rawValue : slice.rawPixels) {
            const double hu = static_cast<double>(rawValue) * slice.rescaleSlope + slice.rescaleIntercept;
            voxels.push_back(clampHu(hu));
        }
    }

    return Result<std::vector<int16_t>>::success(std::move(voxels));
}

[[nodiscard]] VolumeMetadata buildMetadata(const std::vector<DicomSliceInfo>& slices, const std::vector<int16_t>& voxels)
{
    VolumeMetadata metadata;
    metadata.dimensions = {
        static_cast<int>(slices.front().columns),
        static_cast<int>(slices.front().rows),
        static_cast<int>(slices.size()),
    };
    metadata.spacingMm = {
        slices.front().rowSpacingMm,
        slices.front().columnSpacingMm,
        computeSliceSpacingMm(slices),
    };
    metadata.originPatientMm = slices.front().imagePositionPatientMm;
    metadata.rowDirectionPatient = normalize(slices.front().rowDirectionPatient);
    metadata.columnDirectionPatient = normalize(slices.front().columnDirectionPatient);
    metadata.sliceDirectionPatient = normalize(cross(metadata.rowDirectionPatient, metadata.columnDirectionPatient));
    metadata.rescaleSlope = slices.front().rescaleSlope;
    metadata.rescaleIntercept = slices.front().rescaleIntercept;

    const auto [minIt, maxIt] = std::minmax_element(voxels.begin(), voxels.end());
    metadata.minHu = static_cast<int>(*minIt);
    metadata.maxHu = static_cast<int>(*maxIt);
    return metadata;
}

[[nodiscard]] uint64_t fnv1a64Append(uint64_t hash, const void* data, size_t size)
{
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (size_t index = 0; index < size; ++index) {
        hash ^= static_cast<uint64_t>(bytes[index]);
        hash *= 1099511628211ULL;
    }
    return hash;
}

template <typename T>
[[nodiscard]] uint64_t fnv1a64Value(uint64_t hash, const T& value)
{
    return fnv1a64Append(hash, &value, sizeof(T));
}

[[nodiscard]] std::string computeDataHash(
    const VolumeMetadata& metadata,
    const std::vector<DicomSliceInfo>& slices,
    const std::vector<int16_t>& voxels)
{
    uint64_t hash = 1469598103934665603ULL;

    hash = fnv1a64Value(hash, metadata.dimensions.x);
    hash = fnv1a64Value(hash, metadata.dimensions.y);
    hash = fnv1a64Value(hash, metadata.dimensions.z);
    hash = fnv1a64Value(hash, metadata.spacingMm.x);
    hash = fnv1a64Value(hash, metadata.spacingMm.y);
    hash = fnv1a64Value(hash, metadata.spacingMm.z);
    hash = fnv1a64Value(hash, metadata.originPatientMm.x);
    hash = fnv1a64Value(hash, metadata.originPatientMm.y);
    hash = fnv1a64Value(hash, metadata.originPatientMm.z);
    hash = fnv1a64Value(hash, metadata.rowDirectionPatient.x);
    hash = fnv1a64Value(hash, metadata.rowDirectionPatient.y);
    hash = fnv1a64Value(hash, metadata.rowDirectionPatient.z);
    hash = fnv1a64Value(hash, metadata.columnDirectionPatient.x);
    hash = fnv1a64Value(hash, metadata.columnDirectionPatient.y);
    hash = fnv1a64Value(hash, metadata.columnDirectionPatient.z);
    hash = fnv1a64Value(hash, metadata.sliceDirectionPatient.x);
    hash = fnv1a64Value(hash, metadata.sliceDirectionPatient.y);
    hash = fnv1a64Value(hash, metadata.sliceDirectionPatient.z);

    const auto appendString = [&hash](const std::string& value) {
        const uint64_t size = static_cast<uint64_t>(value.size());
        hash = fnv1a64Value(hash, size);
        hash = fnv1a64Append(hash, value.data(), value.size());
    };
    appendString(slices.front().studyUid);
    appendString(slices.front().seriesUid);

    for (int16_t value : voxels) {
        hash = fnv1a64Value(hash, value);
    }

    std::ostringstream out;
    out << std::hex << std::setw(16) << std::setfill('0') << hash;
    return out.str();
}

}  // namespace
#endif

Result<VolumeData> DicomVolumeLoader::loadFolder(const std::filesystem::path& folder) const
{
    if (!std::filesystem::exists(folder) || !std::filesystem::is_directory(folder)) {
        return Result<VolumeData>::failure({
            "DICOM_FOLDER_NOT_FOUND",
            "DICOM folder does not exist.",
            folder.string(),
            true,
        });
    }

#if MEASUREMENT_HAVE_DCMTK
    auto slices = scanCtSlices(folder);
    if (!slices.ok()) {
        return Result<VolumeData>::failure(slices.error());
    }

    auto validation = validateAndSortSlices(slices.value());
    if (!validation.ok()) {
        return Result<VolumeData>::failure(validation.error());
    }

    auto voxels = buildHuVolume(slices.value());
    if (!voxels.ok()) {
        return Result<VolumeData>::failure(voxels.error());
    }
    std::vector<int16_t> huVoxels = std::move(voxels.value());

    VolumeData volume;
    volume.metadata = buildMetadata(slices.value(), huVoxels);
    auto transform = makeVolumeTransform(volume.metadata);
    if (!transform.ok()) {
        return Result<VolumeData>::failure(transform.error());
    }

    volume.transform = transform.value();
    volume.dataHash = computeDataHash(volume.metadata, slices.value(), huVoxels);
    volume.image = std::make_shared<DenseHuVolume>(
        volume.metadata.dimensions,
        std::move(huVoxels));
    volume.patientPositionCode = slices.value().front().patientPositionCode;
    volume.sourceFolder = folder.string();
    volume.seriesUid = slices.value().front().seriesUid;
    volume.studyUid = slices.value().front().studyUid;
    return Result<VolumeData>::success(std::move(volume));
#else
    return Result<VolumeData>::failure({
        "DICOM_DEPENDENCY_MISSING",
        "DCMTK was not found. DICOM import is disabled for this build.",
        "Install DCMTK or fetch the pinned fallback into third_party/dcmtk.",
        true,
    });
#endif
}

}  // namespace measurement
