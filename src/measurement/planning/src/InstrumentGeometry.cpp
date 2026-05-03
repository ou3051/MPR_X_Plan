#include "measurement/planning/InstrumentGeometry.h"

#include <cmath>

namespace measurement {

namespace {

constexpr double kPi = 3.14159265358979323846;

[[nodiscard]] Vec3d choosePerpendicular(Vec3d direction)
{
    const Vec3d axis = std::abs(direction.x) < 0.9 ? Vec3d{1.0, 0.0, 0.0} : Vec3d{0.0, 1.0, 0.0};
    return normalize(cross(direction, axis));
}

[[nodiscard]] MeshData buildCylinder(const Instrument& instrument, int radialSegments)
{
    MeshData mesh;
    if (!validateInstrument(instrument).ok() || radialSegments < 3) {
        return mesh;
    }

    const Vec3d axis = normalize(instrument.directionPatientUnit);
    const Vec3d u = choosePerpendicular(axis);
    const Vec3d v = normalize(cross(axis, u));
    const double radius = instrument.diameterMm * 0.5;
    const Vec3d start = instrument.entryPointPatientMm;
    const Vec3d end = endpointPatientMm(instrument);

    for (int i = 0; i < radialSegments; ++i) {
        const double angle = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(radialSegments);
        const Vec3d radial = u * std::cos(angle) * radius + v * std::sin(angle) * radius;
        mesh.vertices.push_back(start + radial);
        mesh.vertices.push_back(end + radial);
    }

    for (int i = 0; i < radialSegments; ++i) {
        const int next = (i + 1) % radialSegments;
        const unsigned int a = static_cast<unsigned int>(i * 2);
        const unsigned int b = static_cast<unsigned int>(i * 2 + 1);
        const unsigned int c = static_cast<unsigned int>(next * 2);
        const unsigned int d = static_cast<unsigned int>(next * 2 + 1);
        mesh.indices.insert(mesh.indices.end(), {a, b, c, c, b, d});
    }
    return mesh;
}

}  // namespace

MeshData InstrumentGeometryBuilder::buildGuidePinMesh(const Instrument& instrument, int radialSegments) const
{
    return buildCylinder(instrument, radialSegments);
}

MeshData InstrumentGeometryBuilder::buildPedicleScrewMesh(const Instrument& instrument, int radialSegments) const
{
    return buildCylinder(instrument, radialSegments);
}

}  // namespace measurement
