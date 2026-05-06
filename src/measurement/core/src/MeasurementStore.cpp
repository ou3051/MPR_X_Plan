#include "measurement/core/MeasurementStore.h"

#include <algorithm>
#include <utility>

namespace measurement {

MeasurementId MeasurementStore::add(MeasurementAnnotation annotation)
{
    annotation.id = MeasurementId(m_nextId++);
    const MeasurementId id = annotation.id;
    m_items.push_back(std::move(annotation));
    notifyAdded(id);
    return id;
}

bool MeasurementStore::remove(MeasurementId id)
{
    const auto it = findIterator(id);
    if (it == m_items.end()) {
        return false;
    }

    m_items.erase(it);
    notifyRemoved(id);
    return true;
}

bool MeasurementStore::update(const MeasurementAnnotation& annotation)
{
    const auto it = findIterator(annotation.id);
    if (it == m_items.end()) {
        return false;
    }

    *it = annotation;
    notifyChanged(annotation.id);
    return true;
}

bool MeasurementStore::updateDistanceEndpoint(MeasurementId id, int pointIndex, Vec3d point)
{
    const auto it = findIterator(id);
    if (it == m_items.end() || !it->updateDistanceEndpoint(pointIndex, point)) {
        return false;
    }

    notifyChanged(id);
    return true;
}

bool MeasurementStore::updateAngleEndpoint(MeasurementId id, int pointIndex, Vec3d point)
{
    const auto it = findIterator(id);
    if (it == m_items.end() || !it->updateAngleEndpoint(pointIndex, point)) {
        return false;
    }

    notifyChanged(id);
    return true;
}

bool MeasurementStore::rename(MeasurementId id, std::string label)
{
    const auto it = findIterator(id);
    if (it == m_items.end()) {
        return false;
    }

    it->label = std::move(label);
    notifyChanged(id);
    return true;
}

void MeasurementStore::clear()
{
    if (m_items.empty()) {
        return;
    }

    m_items.clear();
    notifyCleared();
}

std::optional<MeasurementAnnotation> MeasurementStore::find(MeasurementId id) const
{
    const auto it = findIterator(id);
    if (it == m_items.end()) {
        return std::nullopt;
    }

    return *it;
}

std::optional<Vec3d> MeasurementStore::anchorWorldPoint(MeasurementId id) const
{
    const auto annotation = find(id);
    if (!annotation.has_value()) {
        return std::nullopt;
    }

    return annotation->anchorWorldPoint();
}

const std::vector<MeasurementAnnotation>& MeasurementStore::all() const
{
    return m_items;
}

std::vector<MeasurementAnnotation> MeasurementStore::list() const
{
    return m_items;
}

size_t MeasurementStore::size() const
{
    return m_items.size();
}

bool MeasurementStore::empty() const
{
    return m_items.empty();
}

void MeasurementStore::addObserver(IMeasurementStoreObserver* observer)
{
    if (observer == nullptr) {
        return;
    }

    if (std::find(m_observers.begin(), m_observers.end(), observer) == m_observers.end()) {
        m_observers.push_back(observer);
    }
}

void MeasurementStore::removeObserver(IMeasurementStoreObserver* observer)
{
    m_observers.erase(std::remove(m_observers.begin(), m_observers.end(), observer), m_observers.end());
}

std::vector<MeasurementAnnotation>::iterator MeasurementStore::findIterator(MeasurementId id)
{
    return std::find_if(m_items.begin(), m_items.end(), [id](const MeasurementAnnotation& item) {
        return item.id == id;
    });
}

std::vector<MeasurementAnnotation>::const_iterator MeasurementStore::findIterator(MeasurementId id) const
{
    return std::find_if(m_items.begin(), m_items.end(), [id](const MeasurementAnnotation& item) {
        return item.id == id;
    });
}

void MeasurementStore::notifyAdded(MeasurementId id)
{
    for (IMeasurementStoreObserver* observer : m_observers) {
        observer->onMeasurementAdded(id);
    }
}

void MeasurementStore::notifyRemoved(MeasurementId id)
{
    for (IMeasurementStoreObserver* observer : m_observers) {
        observer->onMeasurementRemoved(id);
    }
}

void MeasurementStore::notifyChanged(MeasurementId id)
{
    for (IMeasurementStoreObserver* observer : m_observers) {
        observer->onMeasurementChanged(id);
    }
}

void MeasurementStore::notifyCleared()
{
    for (IMeasurementStoreObserver* observer : m_observers) {
        observer->onMeasurementsCleared();
    }
}

}  // namespace measurement
