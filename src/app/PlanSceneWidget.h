#pragma once

#include "measurement/core/Instrument.h"
#include "measurement/core/Volume.h"
#include "measurement/core/Xray.h"

#include <QImage>
#include <QSize>
#include <QWidget>

#include <array>
#include <memory>
#include <string>

namespace measurement_app {

class IPlanSceneView {
public:
    virtual ~IPlanSceneView() = default;

    virtual void setVolume(const measurement::VolumeData* volume) = 0;
    virtual void setPlan(const measurement::SurgicalPlan* plan) = 0;
    virtual void setSelectedInstrumentId(std::string id) = 0;
    virtual void refreshScene() = 0;
};

class PlanSceneWidget final : public QWidget, public IPlanSceneView {
public:
    explicit PlanSceneWidget(QWidget* parent = nullptr);
    ~PlanSceneWidget() override;

    void setVolume(const measurement::VolumeData* volume) override;
    void setPlan(const measurement::SurgicalPlan* plan) override;
    void setSelectedInstrumentId(std::string id) override;
    void setDrrProjections(std::array<measurement::ProjectionParams, 2> projections, std::array<bool, 2> enabled);
    void setDrrImages(std::array<QImage, 2> images);
    void refreshScene() override;
    void resetCamera();

protected:
    QSize minimumSizeHint() const override;

private:
    struct Impl;

    [[nodiscard]] std::string volumeSignature() const;
    void rebuildScene();

    std::unique_ptr<Impl> m_impl;
    const measurement::VolumeData* m_volume = nullptr;
    const measurement::SurgicalPlan* m_plan = nullptr;
    std::string m_selectedInstrumentId;
    std::array<measurement::ProjectionParams, 2> m_drrProjections{};
    std::array<bool, 2> m_drrProjectionEnabled{false, false};
    std::array<QImage, 2> m_drrImages{};
};

}  // namespace measurement_app
