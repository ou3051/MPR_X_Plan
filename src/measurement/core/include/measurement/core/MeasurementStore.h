#pragma once

#include "measurement/core/MeasurementAnnotation.h"

#include <cstdint>
#include <optional>
#include <vector>

namespace measurement {

class IMeasurementStoreObserver {
public:
    virtual ~IMeasurementStoreObserver() = default;

    virtual void onMeasurementAdded(MeasurementId id) = 0;
    virtual void onMeasurementRemoved(MeasurementId id) = 0;
    virtual void onMeasurementChanged(MeasurementId id) = 0;
    virtual void onMeasurementsCleared() = 0;
};

class MeasurementStore {
public:
    [[nodiscard]] MeasurementId add(MeasurementAnnotation annotation);
    [[nodiscard]] bool remove(MeasurementId id);
    [[nodiscard]] bool update(const MeasurementAnnotation& annotation);
    [[nodiscard]] bool updateDistanceEndpoint(MeasurementId id, int pointIndex, Vec3d point);
    [[nodiscard]] bool updateAngleEndpoint(MeasurementId id, int pointIndex, Vec3d point);
    [[nodiscard]] bool rename(MeasurementId id, std::string label);
    void clear();

    [[nodiscard]] std::optional<MeasurementAnnotation> find(MeasurementId id) const;
    [[nodiscard]] std::optional<Vec3d> anchorWorldPoint(MeasurementId id) const;
    [[nodiscard]] const std::vector<MeasurementAnnotation>& all() const;
    [[nodiscard]] std::vector<MeasurementAnnotation> list() const;
    [[nodiscard]] size_t size() const;
    [[nodiscard]] bool empty() const;

    void addObserver(IMeasurementStoreObserver* observer);
    void removeObserver(IMeasurementStoreObserver* observer);

private:
    [[nodiscard]] std::vector<MeasurementAnnotation>::iterator findIterator(MeasurementId id);
    [[nodiscard]] std::vector<MeasurementAnnotation>::const_iterator findIterator(MeasurementId id) const;

    void notifyAdded(MeasurementId id);
    void notifyRemoved(MeasurementId id);
    void notifyChanged(MeasurementId id);
    void notifyCleared();

    std::vector<MeasurementAnnotation> m_items;
    std::vector<IMeasurementStoreObserver*> m_observers;
    std::int64_t m_nextId = 0;
};

}  // namespace measurement
