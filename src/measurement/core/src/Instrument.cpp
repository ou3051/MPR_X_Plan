#include "measurement/core/Instrument.h"

#include <algorithm>
#include <utility>

namespace measurement {

Vec3d endpointPatientMm(const Instrument& instrument)
{
    return instrument.entryPointPatientMm + instrument.directionPatientUnit * instrument.lengthMm;
}

Result<void> validateInstrument(const Instrument& instrument)
{
    if (instrument.id.empty()) {
        return Result<void>::failure({"INSTRUMENT_ID_EMPTY", "Instrument id is empty.", "", true});
    }
    if (instrument.lengthMm <= 0.0) {
        return Result<void>::failure({"INSTRUMENT_LENGTH_INVALID", "Instrument length must be positive.", "", true});
    }
    if (instrument.diameterMm <= 0.0) {
        return Result<void>::failure({"INSTRUMENT_DIAMETER_INVALID", "Instrument diameter must be positive.", "", true});
    }
    if (!nearlyEqual(length(instrument.directionPatientUnit), 1.0, 1.0e-6)) {
        return Result<void>::failure({"INSTRUMENT_DIRECTION_INVALID", "Instrument direction must be normalized.", "", true});
    }
    return Result<void>::success();
}

const std::vector<Instrument>& SurgicalPlan::instruments() const
{
    return m_instruments;
}

Result<void> SurgicalPlan::addInstrument(Instrument instrument)
{
    const Result<void> validation = validateInstrument(instrument);
    if (!validation.ok()) {
        return validation;
    }
    if (findInstrument(instrument.id) != nullptr) {
        return Result<void>::failure({"INSTRUMENT_ID_DUPLICATE", "Instrument id already exists.", instrument.id, true});
    }
    m_instruments.push_back(std::move(instrument));
    return Result<void>::success();
}

Result<void> SurgicalPlan::updateInstrument(const std::string& id, const InstrumentPatch& patch)
{
    Instrument* instrument = findInstrument(id);
    if (instrument == nullptr) {
        return Result<void>::failure({"INSTRUMENT_NOT_FOUND", "Instrument was not found.", id, true});
    }
    if (instrument->locked) {
        return Result<void>::failure({"INSTRUMENT_LOCKED", "Instrument is locked.", id, true});
    }

    Instrument updated = *instrument;
    updated.entryPointPatientMm = patch.entryPointPatientMm;
    updated.directionPatientUnit = normalize(patch.directionPatientUnit);
    updated.lengthMm = patch.lengthMm;
    updated.diameterMm = patch.diameterMm;
    updated.visible = patch.visible;
    updated.locked = patch.locked;
    updated.label = patch.label;

    const Result<void> validation = validateInstrument(updated);
    if (!validation.ok()) {
        return validation;
    }

    *instrument = std::move(updated);
    return Result<void>::success();
}

Result<void> SurgicalPlan::removeInstrument(const std::string& id)
{
    const auto oldSize = m_instruments.size();
    m_instruments.erase(
        std::remove_if(m_instruments.begin(), m_instruments.end(), [&](const Instrument& instrument) {
            return instrument.id == id;
        }),
        m_instruments.end());

    if (m_instruments.size() == oldSize) {
        return Result<void>::failure({"INSTRUMENT_NOT_FOUND", "Instrument was not found.", id, true});
    }
    return Result<void>::success();
}

Instrument* SurgicalPlan::findInstrument(const std::string& id)
{
    const auto it = std::find_if(m_instruments.begin(), m_instruments.end(), [&](const Instrument& instrument) {
        return instrument.id == id;
    });
    return it == m_instruments.end() ? nullptr : &(*it);
}

const Instrument* SurgicalPlan::findInstrument(const std::string& id) const
{
    const auto it = std::find_if(m_instruments.begin(), m_instruments.end(), [&](const Instrument& instrument) {
        return instrument.id == id;
    });
    return it == m_instruments.end() ? nullptr : &(*it);
}

}  // namespace measurement
