#include "measurement/persistence/ProjectManifest.h"

#include <sstream>

namespace measurement {

namespace {

[[nodiscard]] std::string escapeJson(const std::string& value)
{
    std::ostringstream out;
    for (const char ch : value) {
        if (ch == '\\' || ch == '"') {
            out << '\\';
        }
        out << ch;
    }
    return out.str();
}

}  // namespace

std::string serializeProjectManifest(const ProjectManifest& manifest)
{
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
    out << "    \"instrumentCount\": " << manifest.plan.instruments().size() << "\n";
    out << "  },\n";
    out << "  \"xray\": {\n";
    out << "    \"sidMm\": " << manifest.xrayView.projection.sidMm << ",\n";
    out << "    \"sodMm\": " << manifest.xrayView.projection.sodMm << "\n";
    out << "  }\n";
    out << "}\n";
    return out.str();
}

}  // namespace measurement
