#pragma once

#include "measurement/core/MeasurementAnnotation.h"

namespace measurement {

class MeasurementStateMachine {
public:
    void setMode(MeasurementMode mode);
    [[nodiscard]] MeasurementMode mode() const;

    void reset();
    [[nodiscard]] bool addPoint(Vec3d worldPointPatientMm, MeasurementAnnotation& completed);
    [[nodiscard]] const std::vector<Vec3d>& pendingPoints() const;

private:
    [[nodiscard]] size_t requiredPointCount() const;

    MeasurementMode m_mode = MeasurementMode::Navigate;
    std::vector<Vec3d> m_pendingPointsPatientMm;
};

}  // namespace measurement
