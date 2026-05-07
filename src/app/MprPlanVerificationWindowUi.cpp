#include "MprPlanVerificationWindow.h"

#include "XrayDisplayWidget.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QSplitter>
#include <QTabWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <array>
#include <optional>
#include <utility>

namespace measurement_app {
namespace {

[[nodiscard]] int xrayPresetIndex(measurement::XrayPreset preset)
{
    return preset == measurement::XrayPreset::LAT ? 1 : 0;
}

[[nodiscard]] QDoubleSpinBox* makeSpin(double minValue, double maxValue, double value, double step)
{
    auto* spin = new QDoubleSpinBox();
    spin->setRange(minValue, maxValue);
    spin->setDecimals(3);
    spin->setSingleStep(step);
    spin->setValue(value);
    return spin;
}

}  // namespace

void MprPlanVerificationWindow::buildUi()
{
    setWindowTitle("MPR Plan Verification");

    auto* central = new QWidget(this);
    central->setObjectName("AppRoot");
    central->setAttribute(Qt::WA_StyledBackground, true);
    auto* root = new QHBoxLayout(central);
    root->setContentsMargins(12, 12, 12, 8);
    root->setSpacing(0);
    auto* splitter = new QSplitter(Qt::Horizontal, central);
    splitter->setObjectName("MainSplitter");
    root->addWidget(splitter);
    setCentralWidget(central);

    auto* mprPanel = new QWidget(splitter);
    auto* mprLayout = new QGridLayout(mprPanel);
    m_axialView = new MprSliceWidget(measurement::MprPlane::Axial, mprPanel);
    m_sagittalView = new MprSliceWidget(measurement::MprPlane::Sagittal, mprPanel);
    m_coronalView = new MprSliceWidget(measurement::MprPlane::Coronal, mprPanel);
    m_sceneView = new PlanSceneWidget(mprPanel);
    mprLayout->addWidget(m_axialView, 0, 0);
    mprLayout->addWidget(m_sagittalView, 0, 1);
    mprLayout->addWidget(m_coronalView, 1, 0);
    mprLayout->addWidget(m_sceneView, 1, 1);

    auto* xrayPanel = new QWidget(splitter);
    xrayPanel->setObjectName("XrayPanel");
    xrayPanel->setAttribute(Qt::WA_StyledBackground, true);
    auto* xrayLayout = new QVBoxLayout(xrayPanel);
    xrayLayout->setContentsMargins(8, 8, 8, 8);
    xrayLayout->setSpacing(8);
    m_apXrayView = new XrayDisplayWidget("AP X-ray", measurement::XrayPreset::AP, xrayPanel);
    m_latXrayView = new XrayDisplayWidget("LAT X-ray", measurement::XrayPreset::LAT, xrayPanel);
    xrayLayout->addWidget(m_apXrayView, 1);
    xrayLayout->addWidget(m_latXrayView, 1);

    auto* controlPanel = new QWidget(splitter);
    controlPanel->setObjectName("ControlPanel");
    controlPanel->setAttribute(Qt::WA_StyledBackground, true);
    auto* controls = new QVBoxLayout(controlPanel);
    controls->setContentsMargins(8, 8, 8, 8);
    controls->setSpacing(8);

    auto* drrPanel = new QWidget(splitter);
    drrPanel->setObjectName("DrrPanel");
    drrPanel->setAttribute(Qt::WA_StyledBackground, true);
    auto* drrOuterLayout = new QVBoxLayout(drrPanel);
    drrOuterLayout->setContentsMargins(8, 8, 8, 8);
    drrOuterLayout->setSpacing(0);
    auto* drrScrollArea = new QScrollArea(drrPanel);
    drrScrollArea->setWidgetResizable(true);
    drrScrollArea->setFrameShape(QFrame::NoFrame);
    auto* drrContent = new QWidget(drrScrollArea);
    auto* drrPanelLayout = new QVBoxLayout(drrContent);
    drrPanelLayout->setContentsMargins(0, 0, 0, 0);
    drrPanelLayout->setSpacing(8);
    drrScrollArea->setWidget(drrContent);
    drrOuterLayout->addWidget(drrScrollArea);

    auto* workflowGroup = new QGroupBox("Workflow", drrContent);
    auto* loadGrid = new QGridLayout(workflowGroup);
    loadGrid->setHorizontalSpacing(8);
    loadGrid->setVerticalSpacing(8);
    m_loadDicomButton = new QPushButton("Load DICOM", workflowGroup);
    auto* resetViewsButton = new QPushButton("Reset Views", workflowGroup);
    auto* saveButton = new QPushButton("Save .mprproj", workflowGroup);
    m_freeObliqueButton = new QPushButton(m_freeObliqueMode ? "Free oblique: On" : "Free oblique: Off", workflowGroup);
    m_freeObliqueButton->setCheckable(true);
    m_freeObliqueButton->setChecked(m_freeObliqueMode);
    loadGrid->addWidget(m_loadDicomButton, 0, 0, 1, 2);
    loadGrid->addWidget(resetViewsButton, 1, 0);
    loadGrid->addWidget(saveButton, 1, 1);
    loadGrid->addWidget(m_freeObliqueButton, 2, 0, 1, 2);
    drrPanelLayout->addWidget(workflowGroup);

    auto* volumeGroup = new QGroupBox("Volume", drrContent);
    auto* volumeLayout = new QVBoxLayout(volumeGroup);
    m_volumeLabel = new QLabel(volumeGroup);
    m_volumeLabel->setObjectName("VolumeInfoLabel");
    m_volumeLabel->setWordWrap(true);
    volumeLayout->addWidget(m_volumeLabel);
    drrPanelLayout->addWidget(volumeGroup);

    auto* postureGroup = new QGroupBox("Patient position", drrContent);
    auto* postureLayout = new QFormLayout(postureGroup);
    m_patientPostureCombo = new QComboBox(postureGroup);
    m_patientPostureCombo->addItem("Supine", false);
    m_patientPostureCombo->addItem("Prone", true);
    m_headFeetDirectionCombo = new QComboBox(postureGroup);
    m_headFeetDirectionCombo->addItem("Head first", false);
    m_headFeetDirectionCombo->addItem("Feet first", true);
    postureLayout->addRow("Body posture", m_patientPostureCombo);
    postureLayout->addRow("Entry direction", m_headFeetDirectionCombo);
    drrPanelLayout->addWidget(postureGroup);

    auto* crosshairGroup = new QGroupBox("Crosshair voxel", drrContent);
    auto* crosshairLayout = new QGridLayout(crosshairGroup);
    m_xSlider = new QSlider(Qt::Horizontal, crosshairGroup);
    m_ySlider = new QSlider(Qt::Horizontal, crosshairGroup);
    m_zSlider = new QSlider(Qt::Horizontal, crosshairGroup);
    m_xValueLabel = new QLabel(crosshairGroup);
    m_yValueLabel = new QLabel(crosshairGroup);
    m_zValueLabel = new QLabel(crosshairGroup);
    crosshairLayout->addWidget(new QLabel("X", crosshairGroup), 0, 0);
    crosshairLayout->addWidget(m_xSlider, 0, 1);
    crosshairLayout->addWidget(m_xValueLabel, 0, 2);
    crosshairLayout->addWidget(new QLabel("Y", crosshairGroup), 1, 0);
    crosshairLayout->addWidget(m_ySlider, 1, 1);
    crosshairLayout->addWidget(m_yValueLabel, 1, 2);
    crosshairLayout->addWidget(new QLabel("Z", crosshairGroup), 2, 0);
    crosshairLayout->addWidget(m_zSlider, 2, 1);
    crosshairLayout->addWidget(m_zValueLabel, 2, 2);
    drrPanelLayout->addWidget(crosshairGroup);
    // Kept wired for programmatic slice changes. Hidden for now because the
    // MPR views provide direct crosshair manipulation.
    crosshairGroup->setVisible(false);

    auto* planningSplitter = new QSplitter(Qt::Vertical, controlPanel);
    planningSplitter->setChildrenCollapsible(false);
    controls->addWidget(planningSplitter, 1);

    auto* measurementGroup = new QGroupBox("Measurements", controlPanel);
    auto* measurementLayout = new QVBoxLayout(measurementGroup);
    auto* measurementButtons = new QGridLayout();
    measurementButtons->setHorizontalSpacing(8);
    measurementButtons->setVerticalSpacing(8);
    m_measureNavigateButton = new QPushButton("Navigate", measurementGroup);
    m_measureDistanceButton = new QPushButton("Distance", measurementGroup);
    m_measureAngleButton = new QPushButton("Angle", measurementGroup);
    for (QPushButton* button : {m_measureNavigateButton, m_measureDistanceButton, m_measureAngleButton}) {
        button->setCheckable(true);
    }
    measurementButtons->addWidget(m_measureNavigateButton, 0, 0);
    measurementButtons->addWidget(m_measureDistanceButton, 0, 1);
    measurementButtons->addWidget(m_measureAngleButton, 0, 2);
    measurementLayout->addLayout(measurementButtons);
    m_measurementList = new QListWidget(measurementGroup);
    m_measurementList->setMinimumHeight(130);
    measurementLayout->addWidget(m_measurementList, 1);
    auto* measurementForm = new QFormLayout();
    m_measurementLabel = new QLineEdit(measurementGroup);
    measurementForm->addRow("Label", m_measurementLabel);
    measurementLayout->addLayout(measurementForm);
    auto* measurementEditButtons = new QGridLayout();
    auto* deleteMeasurement = new QPushButton("Delete", measurementGroup);
    auto* clearMeasurementsButton = new QPushButton("Clear", measurementGroup);
    measurementEditButtons->addWidget(deleteMeasurement, 0, 0);
    measurementEditButtons->addWidget(clearMeasurementsButton, 0, 1);
    measurementLayout->addLayout(measurementEditButtons);
    planningSplitter->addWidget(measurementGroup);

    auto* instrumentGroup = new QGroupBox("Plan instruments", controlPanel);
    auto* instrumentLayout = new QVBoxLayout(instrumentGroup);
    m_instrumentList = new QListWidget(instrumentGroup);
    m_instrumentList->setMinimumHeight(160);
    instrumentLayout->addWidget(m_instrumentList, 1);

    auto* form = new QFormLayout();
    m_label = new QLineEdit(instrumentGroup);
    m_length = makeSpin(1.0, 300.0, 55.0, 1.0);
    m_diameter = makeSpin(0.5, 20.0, 2.0, 0.5);
    form->addRow("Name", m_label);
    form->addRow("Length mm", m_length);
    form->addRow("Diameter mm", m_diameter);
    instrumentLayout->addLayout(form);

    auto* instrumentButtons = new QGridLayout();
    instrumentButtons->setHorizontalSpacing(8);
    instrumentButtons->setVerticalSpacing(8);
    auto* addPin = new QPushButton("Add pin at crosshair", instrumentGroup);
    auto* addScrew = new QPushButton("Add screw at crosshair", instrumentGroup);
    m_editInstrumentButton = new QPushButton("Edit selected", instrumentGroup);
    auto* remove = new QPushButton("Delete", instrumentGroup);
    instrumentButtons->addWidget(addPin, 0, 0);
    instrumentButtons->addWidget(addScrew, 0, 1);
    instrumentButtons->addWidget(m_editInstrumentButton, 1, 0);
    instrumentButtons->addWidget(remove, 1, 1);
    instrumentLayout->addLayout(instrumentButtons);

    auto* drrPlacementButtons = new QGridLayout();
    drrPlacementButtons->setHorizontalSpacing(8);
    drrPlacementButtons->setVerticalSpacing(8);
    m_drrPinButton = new QPushButton("DRR Pin", instrumentGroup);
    m_drrScrewButton = new QPushButton("DRR Screw", instrumentGroup);
    m_drrCancelButton = new QPushButton("Cancel DRR placement", instrumentGroup);
    m_drrPinButton->setCheckable(true);
    m_drrScrewButton->setCheckable(true);
    drrPlacementButtons->addWidget(m_drrPinButton, 0, 0);
    drrPlacementButtons->addWidget(m_drrScrewButton, 0, 1);
    drrPlacementButtons->addWidget(m_drrCancelButton, 1, 0, 1, 2);
    instrumentLayout->addLayout(drrPlacementButtons);
    planningSplitter->addWidget(instrumentGroup);
    planningSplitter->setSizes({260, 460});
    controlPanel->setMinimumWidth(330);

    auto* statusGroup = new QGroupBox("Status", drrContent);
    auto* statusLayout = new QVBoxLayout(statusGroup);
    m_statusLabel = new QLabel(statusGroup);
    m_statusLabel->setObjectName("StatusInfoLabel");
    m_statusLabel->setWordWrap(true);
    statusLayout->addWidget(m_statusLabel);
    drrPanelLayout->addWidget(statusGroup);
    auto* drrGroup = new QGroupBox("DRR Parameters", drrPanel);
    auto* drrGroupLayout = new QVBoxLayout(drrGroup);
    drrGroup->setTitle("DRR Parameters");
    drrGroupLayout->setContentsMargins(9, 9, 9, 9);
    drrGroupLayout->setSpacing(8);
    auto* drrTabs = new QTabWidget(drrGroup);

    const auto makeDrrParameterPage = [&](measurement::XrayPreset preset, const QString& title) {
        const int index = xrayPresetIndex(preset);
        auto* page = new QWidget(drrTabs);
        auto* drrForm = new QFormLayout(page);

        m_drrSid[index] = makeSpin(2.0, 1.0e6, 1000.0, 10.0);
        m_drrSid[index]->setDecimals(1);
        m_drrSid[index]->setSuffix(" mm");
        m_drrSod[index] = makeSpin(1.0, 1.0e6, 700.0, 10.0);
        m_drrSod[index]->setDecimals(1);
        m_drrSod[index]->setSuffix(" mm");
        m_drrDetectorWidth[index] = makeSpin(32.0, 4096.0, 320.0, 10.0);
        m_drrDetectorWidth[index]->setDecimals(1);
        m_drrDetectorWidth[index]->setSuffix(" mm");
        m_drrDetectorHeight[index] = makeSpin(32.0, 4096.0, 240.0, 10.0);
        m_drrDetectorHeight[index]->setDecimals(1);
        m_drrDetectorHeight[index]->setSuffix(" mm");
        m_drrPixelSpacing[index] = makeSpin(0.0, 1000.0, 0.0, 0.05);
        m_drrPixelSpacing[index]->setDecimals(3);
        m_drrPixelSpacing[index]->setSuffix(" mm");
        m_drrPixelSpacing[index]->setSpecialValueText("Auto");
        m_drrRayStep[index] = makeSpin(0.001, 1000.0, 1.0, 0.1);
        m_drrRayStep[index]->setDecimals(3);
        m_drrRayStep[index]->setSuffix(" mm");
        m_drrWindowCenter[index] = makeSpin(0.0, 1.0e6, 0.0, 10.0);
        m_drrWindowCenter[index]->setDecimals(2);
        m_drrWindowCenter[index]->setSpecialValueText("Auto");
        m_drrWindowWidth[index] = makeSpin(0.0, 1.0e6, 0.0, 10.0);
        m_drrWindowWidth[index]->setDecimals(2);
        m_drrWindowWidth[index]->setSpecialValueText("Auto");
        m_drrGamma[index] = makeSpin(0.1, 10.0, 1.0, 0.1);
        m_drrGamma[index]->setDecimals(2);
        m_drrHuOffset[index] = makeSpin(-10000.0, 10000.0, 0.0, 10.0);
        m_drrHuOffset[index]->setDecimals(1);
        m_drrHuScale[index] = makeSpin(0.001, 100.0, 1.0, 0.05);
        m_drrHuScale[index]->setDecimals(4);

        drrForm->addRow("SID", m_drrSid[index]);
        drrForm->addRow("SOD", m_drrSod[index]);
        drrForm->addRow("Detector width", m_drrDetectorWidth[index]);
        drrForm->addRow("Detector height", m_drrDetectorHeight[index]);
        drrForm->addRow("Pixel spacing", m_drrPixelSpacing[index]);
        drrForm->addRow("Ray step", m_drrRayStep[index]);
        drrForm->addRow("Window center", m_drrWindowCenter[index]);
        drrForm->addRow("Window width", m_drrWindowWidth[index]);
        drrForm->addRow("Gamma", m_drrGamma[index]);
        drrForm->addRow("HU offset", m_drrHuOffset[index]);
        drrForm->addRow("HU scale", m_drrHuScale[index]);
        drrTabs->addTab(page, title);
    };
    makeDrrParameterPage(measurement::XrayPreset::AP, "AP");
    makeDrrParameterPage(measurement::XrayPreset::LAT, "LAT");
    drrGroupLayout->addWidget(drrTabs);
    drrPanelLayout->addWidget(drrGroup);
    drrPanelLayout->addStretch(1);
    drrPanel->setMinimumWidth(280);

    splitter->addWidget(mprPanel);
    splitter->addWidget(xrayPanel);
    splitter->addWidget(controlPanel);
    splitter->addWidget(drrPanel);
    splitter->setChildrenCollapsible(false);
    // The planning column is reserved for measurement and instrument workflows;
    // the rightmost auxiliary column owns project, status, patient, and DRR
    // controls.
    splitter->setStretchFactor(0, 4);
    splitter->setStretchFactor(1, 2);
    splitter->setStretchFactor(2, 2);
    splitter->setStretchFactor(3, 2);
    splitter->setSizes({620, 320, 340, 300});

    const auto crosshairCallback = [this](measurement::Vec3d patient) {
        setCrosshairPatient(patient);
    };
    const auto windowLevelCallback = [this](double centerHu, double widthHu) {
        setWindowLevel(centerHu, widthHu);
    };
    const auto rotationCallback = [this](measurement::MprPlane aroundPlane, measurement::MprPlane linePlane, double deltaAngleRad) {
        rotateCrosshairPlane(aroundPlane, linePlane, deltaAngleRad);
    };
    m_axialView->setCrosshairChangedCallback(crosshairCallback);
    m_sagittalView->setCrosshairChangedCallback(crosshairCallback);
    m_coronalView->setCrosshairChangedCallback(crosshairCallback);
    m_axialView->setWindowLevelChangedCallback(windowLevelCallback);
    m_sagittalView->setWindowLevelChangedCallback(windowLevelCallback);
    m_coronalView->setWindowLevelChangedCallback(windowLevelCallback);
    m_axialView->setPlaneRotationCallback(rotationCallback);
    m_sagittalView->setPlaneRotationCallback(rotationCallback);
    m_coronalView->setPlaneRotationCallback(rotationCallback);
    const auto activatePlaneCallback = [this](measurement::MprPlane plane) {
        activateMprPlane(plane);
    };
    m_axialView->setActivatedCallback(activatePlaneCallback);
    m_sagittalView->setActivatedCallback(activatePlaneCallback);
    m_coronalView->setActivatedCallback(activatePlaneCallback);
    m_axialView->setLinkedPlaneFrames(&m_planeFrames);
    m_sagittalView->setLinkedPlaneFrames(&m_planeFrames);
    m_coronalView->setLinkedPlaneFrames(&m_planeFrames);
    m_axialView->setMeasurements(&m_measurementStore.all());
    m_sagittalView->setMeasurements(&m_measurementStore.all());
    m_coronalView->setMeasurements(&m_measurementStore.all());

    const auto measurementPointCallback = [this](
                                              measurement::MprPlane plane,
                                              measurement::Vec3d patientPoint,
                                              measurement::MeasurementPlane slicePlane) {
        handleMeasurementPointAdded(plane, patientPoint, slicePlane);
    };
    const auto measurementHoverCallback = [this](measurement::MprPlane plane, std::optional<measurement::Vec3d> patientPoint) {
        handleMeasurementHoverChanged(plane, patientPoint);
    };
    const auto measurementCancelCallback = [this]() {
        cancelPendingMeasurement();
    };
    m_axialView->setMeasurementPointAddedCallback(measurementPointCallback);
    m_sagittalView->setMeasurementPointAddedCallback(measurementPointCallback);
    m_coronalView->setMeasurementPointAddedCallback(measurementPointCallback);
    m_axialView->setMeasurementHoverChangedCallback(measurementHoverCallback);
    m_sagittalView->setMeasurementHoverChangedCallback(measurementHoverCallback);
    m_coronalView->setMeasurementHoverChangedCallback(measurementHoverCallback);
    m_axialView->setMeasurementCancelCallback(measurementCancelCallback);
    m_sagittalView->setMeasurementCancelCallback(measurementCancelCallback);
    m_coronalView->setMeasurementCancelCallback(measurementCancelCallback);

    const auto drrLineCompleted = [this](measurement::XrayPreset preset, DrrDetectorLine line) {
        handleDrrLineCompleted(preset, line);
    };
    const auto drrInstrumentSelected = [this](std::string id) {
        handleDrrInstrumentSelected(std::move(id));
    };
    const auto drrInstrumentDragged = [this](
                                          measurement::XrayPreset preset,
                                          std::string id,
                                          DrrInteractionTarget target,
                                          DrrDetectorPoint detectorPoint) {
        handleDrrInstrumentDragged(preset, std::move(id), target, detectorPoint);
    };
    m_apXrayView->setLineCompletedCallback(drrLineCompleted);
    m_latXrayView->setLineCompletedCallback(drrLineCompleted);
    m_apXrayView->setInstrumentSelectedCallback(drrInstrumentSelected);
    m_latXrayView->setInstrumentSelectedCallback(drrInstrumentSelected);
    m_apXrayView->setInstrumentDraggedCallback(drrInstrumentDragged);
    m_latXrayView->setInstrumentDraggedCallback(drrInstrumentDragged);

    connect(m_loadDicomButton, &QPushButton::clicked, this, [this]() { loadDicomFolder(); });
    connect(resetViewsButton, &QPushButton::clicked, this, [this]() { resetAllViews(); });
    connect(saveButton, &QPushButton::clicked, this, [this]() { saveProject(); });
    connect(m_freeObliqueButton, &QPushButton::toggled, this, [this](bool checked) { setFreeObliqueMode(checked); });
    connect(m_measureNavigateButton, &QPushButton::clicked, this, [this]() { setMeasurementMode(measurement::MeasurementMode::Navigate); });
    connect(m_measureDistanceButton, &QPushButton::clicked, this, [this]() { setMeasurementMode(measurement::MeasurementMode::Distance); });
    connect(m_measureAngleButton, &QPushButton::clicked, this, [this]() { setMeasurementMode(measurement::MeasurementMode::Angle); });
    connect(deleteMeasurement, &QPushButton::clicked, this, [this]() { deleteSelectedMeasurement(); });
    connect(clearMeasurementsButton, &QPushButton::clicked, this, [this]() { clearMeasurements(); });
    connect(m_measurementLabel, &QLineEdit::editingFinished, this, [this]() { renameSelectedMeasurement(); });
    const auto measurementSelectionChanged = [this]() {
        if (const auto id = selectedMeasurementId()) {
            selectMeasurementById(*id);
            jumpToMeasurement(*id);
        }
    };
    connect(m_measurementList, &QListWidget::currentRowChanged, this, [measurementSelectionChanged](int) {
        measurementSelectionChanged();
    });
    connect(m_measurementList, &QListWidget::itemClicked, this, [measurementSelectionChanged](QListWidgetItem*) {
        measurementSelectionChanged();
    });
    connect(addPin, &QPushButton::clicked, this, [this]() { addInstrument(measurement::InstrumentType::GuidePin); });
    connect(addScrew, &QPushButton::clicked, this, [this]() { addInstrument(measurement::InstrumentType::PedicleScrew); });
    connect(m_editInstrumentButton, &QPushButton::clicked, this, [this]() { toggleInstrumentEdit(); });
    connect(remove, &QPushButton::clicked, this, [this]() { removeSelectedInstrument(); });
    connect(m_drrPinButton, &QPushButton::clicked, this, [this]() {
        setDrrPlacementMode(measurement::InstrumentType::GuidePin);
    });
    connect(m_drrScrewButton, &QPushButton::clicked, this, [this]() {
        setDrrPlacementMode(measurement::InstrumentType::PedicleScrew);
    });
    connect(m_drrCancelButton, &QPushButton::clicked, this, [this]() { cancelDrrPlacement(); });
    const auto instrumentSelectionChanged = [this]() {
        syncPlacementSelectionFromUi();
        const bool jumped = jumpToInstrumentPlanningPose(selectedInstrumentId());
        syncSpinBoxesFromSelectedInstrument();
        if (!jumped) {
            refreshAll(true);
        }
    };
    connect(m_instrumentList, &QListWidget::currentRowChanged, this, [instrumentSelectionChanged]() {
        instrumentSelectionChanged();
    });
    connect(m_instrumentList, &QListWidget::itemClicked, this, [instrumentSelectionChanged](QListWidgetItem*) {
        instrumentSelectionChanged();
    });
    connect(m_patientPostureCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) {
        const bool prone = m_patientPostureCombo != nullptr
            && m_patientPostureCombo->currentData().toBool();
        const bool feetFirst = m_headFeetDirectionCombo != nullptr
            && m_headFeetDirectionCombo->currentData().toBool();
        applyPatientPosition(prone, feetFirst);
    });
    connect(m_headFeetDirectionCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) {
        const bool prone = m_patientPostureCombo != nullptr
            && m_patientPostureCombo->currentData().toBool();
        const bool feetFirst = m_headFeetDirectionCombo != nullptr
            && m_headFeetDirectionCombo->currentData().toBool();
        applyPatientPosition(prone, feetFirst);
    });

    const auto sliderChanged = [this]() {
        if (m_syncingControls) {
            return;
        }
        setCrosshairVoxel({
            static_cast<double>(m_xSlider->value()),
            static_cast<double>(m_ySlider->value()),
            static_cast<double>(m_zSlider->value()),
        });
    };
    connect(m_xSlider, &QSlider::valueChanged, this, sliderChanged);
    connect(m_ySlider, &QSlider::valueChanged, this, sliderChanged);
    connect(m_zSlider, &QSlider::valueChanged, this, sliderChanged);
    connect(m_label, &QLineEdit::editingFinished, this, [this]() { applyInstrumentPropertyEdits(); });
    connect(m_length, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { applyInstrumentPropertyEdits(); });
    connect(m_diameter, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { applyInstrumentPropertyEdits(); });

    const auto drrSettingChanged = [this](double) {
        QObject* changedControl = sender();
        bool displayOnly = false;
        for (size_t index = 0; index < m_drrWindowCenter.size(); ++index) {
            displayOnly = displayOnly
                || changedControl == m_drrWindowCenter[index]
                || changedControl == m_drrWindowWidth[index]
                || changedControl == m_drrGamma[index];
        }
        for (size_t index = 0; index < m_drrSid.size(); ++index) {
            if (m_drrSid[index] != nullptr && m_drrSod[index] != nullptr && m_drrSod[index]->value() >= m_drrSid[index]->value()) {
                m_drrSod[index]->blockSignals(true);
                m_drrSod[index]->setValue(std::max(1.0, m_drrSid[index]->value() - 1.0));
                m_drrSod[index]->blockSignals(false);
            }
        }
        if (displayOnly) {
            if (m_apXrayView != nullptr) {
                m_apXrayView->setDrrSettings(drrSettingsFromControls(measurement::XrayPreset::AP));
                m_apXrayView->refreshDisplaySettings();
            }
            if (m_latXrayView != nullptr) {
                m_latXrayView->setDrrSettings(drrSettingsFromControls(measurement::XrayPreset::LAT));
                m_latXrayView->refreshDisplaySettings();
            }
            if (m_sceneView != nullptr) {
                std::array<QImage, 2> drrImages{};
                if (m_apXrayView != nullptr) {
                    drrImages[0] = m_apXrayView->renderedImage();
                }
                if (m_latXrayView != nullptr) {
                    drrImages[1] = m_latXrayView->renderedImage();
                }
                m_sceneView->setDrrImages(std::move(drrImages));
                m_sceneView->refreshScene();
            }
            return;
        }
        refreshXrayViews();
    };
    for (size_t index = 0; index < m_drrSid.size(); ++index) {
        connect(m_drrSid[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrSod[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrDetectorWidth[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrDetectorHeight[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrPixelSpacing[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrRayStep[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrWindowCenter[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrWindowWidth[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrGamma[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrHuOffset[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
        connect(m_drrHuScale[index], qOverload<double>(&QDoubleSpinBox::valueChanged), this, drrSettingChanged);
    }

    setMeasurementMode(measurement::MeasurementMode::Navigate);
    refreshMeasurementList();
    resize(1560, 840);
}

}  // namespace measurement_app