#pragma once

#include <optional>
#include <string>
#include <utility>

namespace measurement {

struct ErrorInfo {
    std::string code;
    std::string message;
    std::string detail;
    bool recoverable = true;
};

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
