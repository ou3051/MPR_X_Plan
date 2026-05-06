#include "measurement/core/MeasurementStateMachine.h"

namespace measurement {

void MeasurementStateMachine::setMode(MeasurementMode mode)
{
    if (m_mode == mode) {
        return;
    }

    m_mode = mode;
    m_pendingPointsPatientMm.clear();
}

MeasurementMode MeasurementStateMachine::mode() const
{
    return m_mode;
}

void MeasurementStateMachine::reset()
{
    m_pendingPointsPatientMm.clear();
}

bool MeasurementStateMachine::addPoint(Vec3d worldPointPatientMm, MeasurementAnnotation& completed)
{
    if (m_mode != MeasurementMode::Distance && m_mode != MeasurementMode::Angle) {
        return false;
    }

    m_pendingPointsPatientMm.push_back(worldPointPatientMm);
    if (m_pendingPointsPatientMm.size() < requiredPointCount()) {
        return false;
    }

    if (m_mode == MeasurementMode::Distance) {
        completed = MeasurementAnnotation::makeDistance(m_pendingPointsPatientMm[0], m_pendingPointsPatientMm[1]);
    } else {
        const auto angle = MeasurementAnnotation::tryMakeAngle(
            m_pendingPointsPatientMm[0],
            m_pendingPointsPatientMm[1],
            m_pendingPointsPatientMm[2],
            m_pendingPointsPatientMm[3]);
        m_pendingPointsPatientMm.clear();
        if (!angle.has_value()) {
            return false;
        }
        completed = *angle;
        return true;
    }

    m_pendingPointsPatientMm.clear();
    return true;
}

const std::vector<Vec3d>& MeasurementStateMachine::pendingPoints() const
{
    return m_pendingPointsPatientMm;
}

size_t MeasurementStateMachine::requiredPointCount() const
{
    switch (m_mode) {
    case MeasurementMode::Distance:
        return 2;
    case MeasurementMode::Angle:
        return 4;
    case MeasurementMode::Navigate:
        return 0;
    }
    return 0;
}

}  // namespace measurement
