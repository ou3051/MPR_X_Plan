#include "measurement/persistence/ProjectManifest.h"

#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

namespace measurement {

namespace {

struct JsonValue {
    enum class Type {
        Null,
        Object,
        Array,
        String,
        Number,
        Bool
    };

    Type type = Type::Null;
    std::map<std::string, JsonValue> object;
    std::vector<JsonValue> array;
    std::string string;
    double number = 0.0;
    bool boolean = false;
};

class JsonParser {
public:
    explicit JsonParser(std::string_view text)
        : m_text(text)
    {
    }

    [[nodiscard]] JsonValue parse()
    {
        JsonValue value = parseValue();
        skipWhitespace();
        if (m_pos != m_text.size()) {
            throw std::runtime_error("Unexpected trailing JSON content.");
        }
        return value;
    }

private:
    [[nodiscard]] JsonValue parseValue()
    {
        skipWhitespace();
        if (m_pos >= m_text.size()) {
            throw std::runtime_error("Unexpected end of JSON.");
        }

        const char ch = m_text[m_pos];
        if (ch == '{') {
            return parseObject();
        }
        if (ch == '[') {
            return parseArray();
        }
        if (ch == '"') {
            JsonValue value;
            value.type = JsonValue::Type::String;
            value.string = parseString();
            return value;
        }
        if (ch == '-' || std::isdigit(static_cast<unsigned char>(ch)) != 0) {
            return parseNumber();
        }
        if (consume("true")) {
            JsonValue value;
            value.type = JsonValue::Type::Bool;
            value.boolean = true;
            return value;
        }
        if (consume("false")) {
            JsonValue value;
            value.type = JsonValue::Type::Bool;
            value.boolean = false;
            return value;
        }
        if (consume("null")) {
            return {};
        }
        throw std::runtime_error("Unexpected JSON token.");
    }

    [[nodiscard]] JsonValue parseObject()
    {
        expect('{');
        JsonValue value;
        value.type = JsonValue::Type::Object;
        skipWhitespace();
        if (peek('}')) {
            expect('}');
            return value;
        }

        while (true) {
            skipWhitespace();
            const std::string key = parseString();
            skipWhitespace();
            expect(':');
            value.object.emplace(key, parseValue());
            skipWhitespace();
            if (peek('}')) {
                expect('}');
                return value;
            }
            expect(',');
        }
    }

    [[nodiscard]] JsonValue parseArray()
    {
        expect('[');
        JsonValue value;
        value.type = JsonValue::Type::Array;
        skipWhitespace();
        if (peek(']')) {
            expect(']');
            return value;
        }

        while (true) {
            value.array.push_back(parseValue());
            skipWhitespace();
            if (peek(']')) {
                expect(']');
                return value;
            }
            expect(',');
        }
    }

    [[nodiscard]] std::string parseString()
    {
        expect('"');
        std::string value;
        while (m_pos < m_text.size()) {
            const char ch = m_text[m_pos++];
            if (ch == '"') {
                return value;
            }
            if (ch != '\\') {
                value.push_back(ch);
                continue;
            }
            if (m_pos >= m_text.size()) {
                throw std::runtime_error("Unterminated JSON escape.");
            }
            const char escaped = m_text[m_pos++];
            switch (escaped) {
            case '"':
            case '\\':
            case '/':
                value.push_back(escaped);
                break;
            case 'b':
                value.push_back('\b');
                break;
            case 'f':
                value.push_back('\f');
                break;
            case 'n':
                value.push_back('\n');
                break;
            case 'r':
                value.push_back('\r');
                break;
            case 't':
                value.push_back('\t');
                break;
            default:
                throw std::runtime_error("Unsupported JSON escape.");
            }
        }
        throw std::runtime_error("Unterminated JSON string.");
    }

    [[nodiscard]] JsonValue parseNumber()
    {
        const size_t begin = m_pos;
        if (peek('-')) {
            ++m_pos;
        }
        while (m_pos < m_text.size() && std::isdigit(static_cast<unsigned char>(m_text[m_pos])) != 0) {
            ++m_pos;
        }
        if (peek('.')) {
            ++m_pos;
            while (m_pos < m_text.size() && std::isdigit(static_cast<unsigned char>(m_text[m_pos])) != 0) {
                ++m_pos;
            }
        }
        if (m_pos < m_text.size() && (m_text[m_pos] == 'e' || m_text[m_pos] == 'E')) {
            ++m_pos;
            if (m_pos < m_text.size() && (m_text[m_pos] == '+' || m_text[m_pos] == '-')) {
                ++m_pos;
            }
            while (m_pos < m_text.size() && std::isdigit(static_cast<unsigned char>(m_text[m_pos])) != 0) {
                ++m_pos;
            }
        }

        JsonValue value;
        value.type = JsonValue::Type::Number;
        value.number = std::stod(std::string(m_text.substr(begin, m_pos - begin)));
        return value;
    }

    void skipWhitespace()
    {
        while (m_pos < m_text.size() && std::isspace(static_cast<unsigned char>(m_text[m_pos])) != 0) {
            ++m_pos;
        }
    }

    [[nodiscard]] bool consume(std::string_view token)
    {
        if (m_text.substr(m_pos, token.size()) != token) {
            return false;
        }
        m_pos += token.size();
        return true;
    }

    [[nodiscard]] bool peek(char expected) const
    {
        return m_pos < m_text.size() && m_text[m_pos] == expected;
    }

    void expect(char expected)
    {
        skipWhitespace();
        if (!peek(expected)) {
            throw std::runtime_error("Unexpected JSON character.");
        }
        ++m_pos;
    }

    std::string_view m_text;
    size_t m_pos = 0;
};

[[nodiscard]] ErrorInfo projectError(std::string code, std::string message, std::string detail = {})
{
    return makeErrorInfo(std::move(code), std::move(message), std::move(detail), true);
}

[[nodiscard]] std::string escapeJson(const std::string& value)
{
    std::ostringstream out;
    for (const char ch : value) {
        switch (ch) {
        case '\\':
            out << "\\\\";
            break;
        case '"':
            out << "\\\"";
            break;
        case '\n':
            out << "\\n";
            break;
        case '\r':
            out << "\\r";
            break;
        case '\t':
            out << "\\t";
            break;
        default:
            out << ch;
            break;
        }
    }
    return out.str();
}

[[nodiscard]] std::string numberJson(double value)
{
    std::ostringstream out;
    out << std::setprecision(17) << value;
    return out.str();
}

void writeVec3(std::ostringstream& out, Vec3d value)
{
    out << "[" << numberJson(value.x) << ", " << numberJson(value.y) << ", " << numberJson(value.z) << "]";
}

[[nodiscard]] std::string instrumentTypeJson(InstrumentType type)
{
    switch (type) {
    case InstrumentType::GuidePin:
        return "GuidePin";
    case InstrumentType::PedicleScrew:
        return "PedicleScrew";
    }
    return "GuidePin";
}

[[nodiscard]] std::optional<InstrumentType> parseInstrumentType(const std::string& value)
{
    if (value == "GuidePin") {
        return InstrumentType::GuidePin;
    }
    if (value == "PedicleScrew") {
        return InstrumentType::PedicleScrew;
    }
    return std::nullopt;
}

[[nodiscard]] std::string xrayPresetJson(XrayPreset preset)
{
    switch (preset) {
    case XrayPreset::AP:
        return "AP";
    case XrayPreset::LAT:
        return "LAT";
    case XrayPreset::Oblique:
        return "Oblique";
    case XrayPreset::Custom:
        return "Custom";
    }
    return "AP";
}

[[nodiscard]] std::optional<XrayPreset> parseXrayPreset(const std::string& value)
{
    if (value == "AP") {
        return XrayPreset::AP;
    }
    if (value == "LAT") {
        return XrayPreset::LAT;
    }
    if (value == "Oblique") {
        return XrayPreset::Oblique;
    }
    if (value == "Custom") {
        return XrayPreset::Custom;
    }
    return std::nullopt;
}

[[nodiscard]] const JsonValue* findMember(const JsonValue& value, const std::string& key)
{
    if (value.type != JsonValue::Type::Object) {
        return nullptr;
    }
    const auto it = value.object.find(key);
    return it == value.object.end() ? nullptr : &it->second;
}

[[nodiscard]] const JsonValue& requireMember(const JsonValue& value, const std::string& key)
{
    const JsonValue* member = findMember(value, key);
    if (member == nullptr) {
        throw std::runtime_error("Missing required field: " + key);
    }
    return *member;
}

[[nodiscard]] std::string requireString(const JsonValue& value, const std::string& key)
{
    const JsonValue& member = requireMember(value, key);
    if (member.type != JsonValue::Type::String) {
        throw std::runtime_error("Field must be a string: " + key);
    }
    return member.string;
}

[[nodiscard]] double requireNumber(const JsonValue& value, const std::string& key)
{
    const JsonValue& member = requireMember(value, key);
    if (member.type != JsonValue::Type::Number) {
        throw std::runtime_error("Field must be a number: " + key);
    }
    return member.number;
}

[[nodiscard]] bool requireBool(const JsonValue& value, const std::string& key)
{
    const JsonValue& member = requireMember(value, key);
    if (member.type != JsonValue::Type::Bool) {
        throw std::runtime_error("Field must be a boolean: " + key);
    }
    return member.boolean;
}

[[nodiscard]] Vec3d requireVec3(const JsonValue& value, const std::string& key)
{
    const JsonValue& member = requireMember(value, key);
    if (member.type != JsonValue::Type::Array || member.array.size() != 3U) {
        throw std::runtime_error("Field must be a vec3 array: " + key);
    }
    for (const JsonValue& item : member.array) {
        if (item.type != JsonValue::Type::Number) {
            throw std::runtime_error("Vec3 array must contain numbers: " + key);
        }
    }
    return {member.array[0].number, member.array[1].number, member.array[2].number};
}

void writeInstrument(std::ostringstream& out, const Instrument& instrument, int indent)
{
    const std::string pad(static_cast<size_t>(indent), ' ');
    out << pad << "{\n";
    out << pad << "  \"id\": \"" << escapeJson(instrument.id) << "\",\n";
    out << pad << "  \"type\": \"" << instrumentTypeJson(instrument.type) << "\",\n";
    out << pad << "  \"entryPointPatientMm\": ";
    writeVec3(out, instrument.entryPointPatientMm);
    out << ",\n";
    out << pad << "  \"directionPatientUnit\": ";
    writeVec3(out, instrument.directionPatientUnit);
    out << ",\n";
    out << pad << "  \"lengthMm\": " << numberJson(instrument.lengthMm) << ",\n";
    out << pad << "  \"diameterMm\": " << numberJson(instrument.diameterMm) << ",\n";
    out << pad << "  \"visible\": " << (instrument.visible ? "true" : "false") << ",\n";
    out << pad << "  \"locked\": " << (instrument.locked ? "true" : "false") << ",\n";
    out << pad << "  \"label\": \"" << escapeJson(instrument.label) << "\"\n";
    out << pad << "}";
}

void writeProjection(std::ostringstream& out, const ProjectionParams& projection, int indent)
{
    const std::string pad(static_cast<size_t>(indent), ' ');
    out << pad << "{\n";
    out << pad << "  \"sourcePosPatientMm\": ";
    writeVec3(out, projection.sourcePosPatientMm);
    out << ",\n";
    out << pad << "  \"detectorCenterPatientMm\": ";
    writeVec3(out, projection.detectorCenterPatientMm);
    out << ",\n";
    out << pad << "  \"detectorUPatientUnit\": ";
    writeVec3(out, projection.detectorUPatientUnit);
    out << ",\n";
    out << pad << "  \"detectorVPatientUnit\": ";
    writeVec3(out, projection.detectorVPatientUnit);
    out << ",\n";
    out << pad << "  \"pixelSpacingMm\": " << numberJson(projection.pixelSpacingMm) << ",\n";
    out << pad << "  \"detectorWidth\": " << projection.detectorWidth << ",\n";
    out << pad << "  \"detectorHeight\": " << projection.detectorHeight << ",\n";
    out << pad << "  \"primaryAngleDeg\": " << numberJson(projection.primaryAngleDeg) << ",\n";
    out << pad << "  \"secondaryAngleDeg\": " << numberJson(projection.secondaryAngleDeg) << ",\n";
    out << pad << "  \"sidMm\": " << numberJson(projection.sidMm) << ",\n";
    out << pad << "  \"sodMm\": " << numberJson(projection.sodMm) << "\n";
    out << pad << "}";
}

void writeXrayView(std::ostringstream& out, const XrayView& view, int indent)
{
    const std::string pad(static_cast<size_t>(indent), ' ');
    out << pad << "{\n";
    out << pad << "  \"preset\": \"" << xrayPresetJson(view.preset) << "\",\n";
    out << pad << "  \"projection\": ";
    writeProjection(out, view.projection, indent + 2);
    out << ",\n";
    out << pad << "  \"windowCenter\": " << numberJson(view.windowCenter) << ",\n";
    out << pad << "  \"windowWidth\": " << numberJson(view.windowWidth) << ",\n";
    out << pad << "  \"showInstrumentOverlay\": " << (view.showInstrumentOverlay ? "true" : "false") << "\n";
    out << pad << "}";
}

[[nodiscard]] Instrument parseInstrument(const JsonValue& value)
{
    if (value.type != JsonValue::Type::Object) {
        throw std::runtime_error("Instrument entry must be an object.");
    }
    const std::optional<InstrumentType> type = parseInstrumentType(requireString(value, "type"));
    if (!type.has_value()) {
        throw std::runtime_error("Unknown instrument type.");
    }

    Instrument instrument;
    instrument.id = requireString(value, "id");
    instrument.type = *type;
    instrument.entryPointPatientMm = requireVec3(value, "entryPointPatientMm");
    instrument.directionPatientUnit = requireVec3(value, "directionPatientUnit");
    instrument.lengthMm = requireNumber(value, "lengthMm");
    instrument.diameterMm = requireNumber(value, "diameterMm");
    instrument.visible = requireBool(value, "visible");
    instrument.locked = requireBool(value, "locked");
    instrument.label = requireString(value, "label");
    return instrument;
}

[[nodiscard]] ProjectionParams parseProjection(const JsonValue& value)
{
    if (value.type != JsonValue::Type::Object) {
        throw std::runtime_error("Projection must be an object.");
    }

    ProjectionParams projection;
    projection.sourcePosPatientMm = requireVec3(value, "sourcePosPatientMm");
    projection.detectorCenterPatientMm = requireVec3(value, "detectorCenterPatientMm");
    projection.detectorUPatientUnit = requireVec3(value, "detectorUPatientUnit");
    projection.detectorVPatientUnit = requireVec3(value, "detectorVPatientUnit");
    projection.pixelSpacingMm = requireNumber(value, "pixelSpacingMm");
    projection.detectorWidth = static_cast<int>(requireNumber(value, "detectorWidth"));
    projection.detectorHeight = static_cast<int>(requireNumber(value, "detectorHeight"));
    projection.primaryAngleDeg = requireNumber(value, "primaryAngleDeg");
    projection.secondaryAngleDeg = requireNumber(value, "secondaryAngleDeg");
    projection.sidMm = requireNumber(value, "sidMm");
    projection.sodMm = requireNumber(value, "sodMm");
    return projection;
}

[[nodiscard]] XrayView parseXrayView(const JsonValue& value)
{
    if (value.type != JsonValue::Type::Object) {
        throw std::runtime_error("Xray view must be an object.");
    }
    const std::optional<XrayPreset> preset = parseXrayPreset(requireString(value, "preset"));
    if (!preset.has_value()) {
        throw std::runtime_error("Unknown Xray preset.");
    }

    XrayView view;
    view.preset = *preset;
    view.projection = parseProjection(requireMember(value, "projection"));
    view.windowCenter = requireNumber(value, "windowCenter");
    view.windowWidth = requireNumber(value, "windowWidth");
    view.showInstrumentOverlay = requireBool(value, "showInstrumentOverlay");
    return view;
}

[[nodiscard]] ProjectManifest parseManifestValue(const JsonValue& value)
{
    if (value.type != JsonValue::Type::Object) {
        throw std::runtime_error("Manifest root must be an object.");
    }

    ProjectManifest manifest;
    manifest.schemaVersion = requireString(value, "schemaVersion");
    if (manifest.schemaVersion != "0.1") {
        throw std::runtime_error("Unsupported project schemaVersion.");
    }
    manifest.softwareVersion = requireString(value, "softwareVersion");

    const JsonValue& dicom = requireMember(value, "dicom");
    manifest.dicomSourceFolder = requireString(dicom, "sourceFolder");
    manifest.studyUid = requireString(dicom, "studyUid");
    manifest.seriesUid = requireString(dicom, "seriesUid");
    manifest.dataHash = requireString(dicom, "dataHash");

    const JsonValue& plan = requireMember(value, "plan");
    const JsonValue& instruments = requireMember(plan, "instruments");
    if (instruments.type != JsonValue::Type::Array) {
        throw std::runtime_error("plan.instruments must be an array.");
    }
    for (const JsonValue& item : instruments.array) {
        const Result<void> addResult = manifest.plan.addInstrument(parseInstrument(item));
        if (!addResult.ok()) {
            throw std::runtime_error(addResult.error().message);
        }
    }

    manifest.xrayView = parseXrayView(requireMember(value, "xrayView"));

    if (const JsonValue* xrayViews = findMember(value, "xrayViews"); xrayViews != nullptr) {
        if (xrayViews->type != JsonValue::Type::Array) {
            throw std::runtime_error("xrayViews must be an array.");
        }
        for (const JsonValue& item : xrayViews->array) {
            manifest.xrayViews.push_back(parseXrayView(item));
        }
    }

    const JsonValue& mprView = requireMember(value, "mprView");
    manifest.mprView.crosshairPatientMm = requireVec3(mprView, "crosshairPatientMm");
    manifest.mprView.zoom = requireNumber(mprView, "zoom");
    manifest.mprView.windowCenterHu = requireNumber(mprView, "windowCenterHu");
    manifest.mprView.windowWidthHu = requireNumber(mprView, "windowWidthHu");

    const JsonValue& view3d = requireMember(value, "view3d");
    manifest.view3d.cameraPositionPatientMm = requireVec3(view3d, "cameraPositionPatientMm");
    manifest.view3d.cameraFocalPointPatientMm = requireVec3(view3d, "cameraFocalPointPatientMm");
    manifest.view3d.cameraUpPatientUnit = requireVec3(view3d, "cameraUpPatientUnit");
    manifest.view3d.zoom = requireNumber(view3d, "zoom");

    return manifest;
}

[[nodiscard]] std::string readTextFile(const std::filesystem::path& path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Unable to open project file.");
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

void replaceFile(const std::filesystem::path& source, const std::filesystem::path& destination)
{
#ifdef _WIN32
    if (!MoveFileExW(
            source.wstring().c_str(),
            destination.wstring().c_str(),
            MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
        throw std::runtime_error("Unable to replace project file.");
    }
#else
    std::filesystem::rename(source, destination);
#endif
}

}  // namespace

std::string serializeProjectManifest(const ProjectManifest& manifest)
{
    const std::vector<XrayView> savedXrayViews = manifest.xrayViews.empty()
        ? std::vector<XrayView>{manifest.xrayView}
        : manifest.xrayViews;

    std::ostringstream out;
    out << "{\n";
    out << "  \"schemaVersion\": \"" << escapeJson(manifest.schemaVersion) << "\",\n";
    out << "  \"softwareVersion\": \"" << escapeJson(manifest.softwareVersion) << "\",\n";
    out << "  \"dicom\": {\n";
    out << "    \"sourceFolder\": \"" << escapeJson(manifest.dicomSourceFolder) << "\",\n";
    out << "    \"studyUid\": \"" << escapeJson(manifest.studyUid) << "\",\n";
    out << "    \"seriesUid\": \"" << escapeJson(manifest.seriesUid) << "\",\n";
    out << "    \"dataHash\": \"" << escapeJson(manifest.dataHash) << "\"\n";
    out << "  },\n";
    out << "  \"plan\": {\n";
    out << "    \"instruments\": [\n";
    for (size_t i = 0; i < manifest.plan.instruments().size(); ++i) {
        writeInstrument(out, manifest.plan.instruments()[i], 6);
        out << (i + 1U == manifest.plan.instruments().size() ? "\n" : ",\n");
    }
    out << "    ]\n";
    out << "  },\n";
    out << "  \"xrayView\": ";
    writeXrayView(out, manifest.xrayView, 2);
    out << ",\n";
    out << "  \"xrayViews\": [\n";
    for (size_t i = 0; i < savedXrayViews.size(); ++i) {
        writeXrayView(out, savedXrayViews[i], 4);
        out << (i + 1U == savedXrayViews.size() ? "\n" : ",\n");
    }
    out << "  ],\n";
    out << "  \"mprView\": {\n";
    out << "    \"crosshairPatientMm\": ";
    writeVec3(out, manifest.mprView.crosshairPatientMm);
    out << ",\n";
    out << "    \"zoom\": " << numberJson(manifest.mprView.zoom) << ",\n";
    out << "    \"windowCenterHu\": " << numberJson(manifest.mprView.windowCenterHu) << ",\n";
    out << "    \"windowWidthHu\": " << numberJson(manifest.mprView.windowWidthHu) << "\n";
    out << "  },\n";
    out << "  \"view3d\": {\n";
    out << "    \"cameraPositionPatientMm\": ";
    writeVec3(out, manifest.view3d.cameraPositionPatientMm);
    out << ",\n";
    out << "    \"cameraFocalPointPatientMm\": ";
    writeVec3(out, manifest.view3d.cameraFocalPointPatientMm);
    out << ",\n";
    out << "    \"cameraUpPatientUnit\": ";
    writeVec3(out, manifest.view3d.cameraUpPatientUnit);
    out << ",\n";
    out << "    \"zoom\": " << numberJson(manifest.view3d.zoom) << "\n";
    out << "  }\n";
    out << "}\n";
    return out.str();
}

Result<ProjectManifest> deserializeProjectManifest(const std::string& json)
{
    try {
        JsonParser parser(json);
        return Result<ProjectManifest>::success(parseManifestValue(parser.parse()));
    } catch (const std::exception& ex) {
        return Result<ProjectManifest>::failure(projectError(
            "PROJECT_MANIFEST_INVALID",
            "Project manifest is invalid.",
            ex.what()));
    }
}

std::string serializeProjectFile(const ProjectManifest& manifest)
{
    std::ostringstream out;
    out << "{\n";
    out << "  \"container\": \"mprproj\",\n";
    out << "  \"entries\": {\n";
    out << "    \"manifest.json\": " << serializeProjectManifest(manifest);
    out << "  }\n";
    out << "}\n";
    return out.str();
}

Result<void> saveProjectFile(const ProjectManifest& manifest, const std::string& path)
{
    try {
        const std::filesystem::path destination(path);
        const std::filesystem::path parent = destination.parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent);
        }

        const std::filesystem::path temporary = destination.string() + ".tmp";
        {
            std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
            if (!output) {
                return Result<void>::failure(projectError(
                    "PROJECT_FILE_WRITE_FAILED",
                    "Unable to write temporary project file.",
                    temporary.string()));
            }
            output << serializeProjectFile(manifest);
        }

        replaceFile(temporary, destination);
        return Result<void>::success();
    } catch (const std::exception& ex) {
        return Result<void>::failure(projectError(
            "PROJECT_FILE_WRITE_FAILED",
            "Unable to save project file.",
            ex.what()));
    }
}

Result<ProjectManifest> loadProjectFile(const std::string& path)
{
    try {
        const std::string text = readTextFile(path);
        JsonParser parser(text);
        const JsonValue root = parser.parse();

        if (const JsonValue* entries = findMember(root, "entries"); entries != nullptr) {
            const JsonValue& manifest = requireMember(*entries, "manifest.json");
            return Result<ProjectManifest>::success(parseManifestValue(manifest));
        }

        return Result<ProjectManifest>::success(parseManifestValue(root));
    } catch (const std::exception& ex) {
        return Result<ProjectManifest>::failure(projectError(
            "PROJECT_FILE_READ_FAILED",
            "Unable to load project file.",
            ex.what()));
    }
}

}  // namespace measurement
