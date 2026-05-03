#pragma once

#include "measurement/core/Geometry.h"
#include "measurement/core/Instrument.h"

#include <vector>

namespace measurement {

struct MeshData {
    std::vector<Vec3d> vertices;
    std::vector<unsigned int> indices;
};

class InstrumentGeometryBuilder {
public:
    [[nodiscard]] MeshData buildGuidePinMesh(const Instrument& instrument, int radialSegments = 24) const;
    [[nodiscard]] MeshData buildPedicleScrewMesh(const Instrument& instrument, int radialSegments = 24) const;
};

}  // namespace measurement
