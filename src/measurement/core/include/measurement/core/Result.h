#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <utility>

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

[[nodiscard]] inline ErrorInfo makeErrorInfo(
    std::string code,
    std::string message,
    std::string detail = {},
    bool recoverable = true)
{
    return {std::move(code), std::move(message), std::move(detail), recoverable};
}

template <typename T>
class Result {
public:
    static Result success(T value)
    {
        Result result;
        result.m_value = std::move(value);
        return result;
    }

    static Result failure(ErrorInfo error)
    {
        Result result;
        result.m_error = std::move(error);
        return result;
    }

    [[nodiscard]] bool ok() const { return m_value.has_value(); }
    [[nodiscard]] const T& value() const { return *m_value; }
    [[nodiscard]] T& value() { return *m_value; }
    [[nodiscard]] const ErrorInfo& error() const { return *m_error; }

private:
    std::optional<T> m_value;
    std::optional<ErrorInfo> m_error;
};

template <>
class Result<void> {
public:
    static Result success() { return Result(true, {}); }
    static Result failure(ErrorInfo error) { return Result(false, std::move(error)); }

    [[nodiscard]] bool ok() const { return m_ok; }
    [[nodiscard]] const ErrorInfo& error() const { return m_error; }

private:
    Result(bool ok, ErrorInfo error)
        : m_ok(ok)
        , m_error(std::move(error))
    {
    }

    bool m_ok = false;
    ErrorInfo m_error;
};

}  // namespace measurement
