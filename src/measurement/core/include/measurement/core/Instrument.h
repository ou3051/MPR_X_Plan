#pragma once

#include "measurement/core/Geometry.h"
#include "measurement/core/Result.h"

#include <string>
#include <vector>

namespace measurement {

enum class InstrumentType {
    GuidePin,
    PedicleScrew
};

struct Instrument {
    std::string id;
    InstrumentType type = InstrumentType::GuidePin;
    Vec3d entryPointPatientMm;
    Vec3d directionPatientUnit{0.0, 0.0, 1.0};
    double lengthMm = 0.0;
    double diameterMm = 0.0;
    bool visible = true;
    bool locked = false;
    std::string label;
};

struct InstrumentPatch {
    Vec3d entryPointPatientMm;
    Vec3d directionPatientUnit;
    double lengthMm = 0.0;
    double diameterMm = 0.0;
    bool visible = true;
    bool locked = false;
    std::string label;
};

[[nodiscard]] Vec3d endpointPatientMm(const Instrument& instrument);
[[nodiscard]] Result<void> validateInstrument(const Instrument& instrument);

class SurgicalPlan {
public:
    [[nodiscard]] const std::vector<Instrument>& instruments() const;
    [[nodiscard]] Result<void> addInstrument(Instrument instrument);
    [[nodiscard]] Result<void> updateInstrument(const std::string& id, const InstrumentPatch& patch);
    [[nodiscard]] Result<void> removeInstrument(const std::string& id);
    [[nodiscard]] Instrument* findInstrument(const std::string& id);
    [[nodiscard]] const Instrument* findInstrument(const std::string& id) const;

private:
    std::vector<Instrument> m_instruments;
};

}  // namespace measurement
