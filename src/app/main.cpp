#include "MprPlanVerificationWindow.h"

#include <QApplication>
#include <QColor>
#include <QCommandLineParser>
#include <QFont>
#include <QPalette>
#include <QString>
#include <QSurfaceFormat>

#include <QVTKOpenGLNativeWidget.h>

namespace {

void applySoftBlueTheme(QApplication& app)
{
    QFont font = app.font();
    font.setFamily("Microsoft YaHei UI");
    font.setPointSize(10);
    app.setFont(font);

    QPalette palette;
    palette.setColor(QPalette::Window, QColor("#edf7ff"));
    palette.setColor(QPalette::WindowText, QColor("#18354f"));
    palette.setColor(QPalette::Base, QColor("#fbfdff"));
    palette.setColor(QPalette::AlternateBase, QColor("#eef7ff"));
    palette.setColor(QPalette::Text, QColor("#18354f"));
    palette.setColor(QPalette::Button, QColor("#d9efff"));
    palette.setColor(QPalette::ButtonText, QColor("#164465"));
    palette.setColor(QPalette::Highlight, QColor("#66b8e8"));
    palette.setColor(QPalette::HighlightedText, QColor("#ffffff"));
    app.setPalette(palette);

    app.setStyleSheet(QStringLiteral(R"(
        QMainWindow {
            background: #edf7ff;
        }

        QWidget {
            color: #18354f;
            font-size: 10pt;
        }

        QWidget#AppRoot {
            background: #edf7ff;
        }

        QWidget#XrayPanel,
        QWidget#ControlPanel,
        QWidget#DrrPanel {
            background: #f7fcff;
            border: 1px solid #c7e2f4;
            border-radius: 8px;
        }

        QSplitter::handle {
            background: #cfe7f8;
            border-radius: 3px;
            margin: 5px 2px;
        }

        QGroupBox {
            background: #fbfdff;
            border: 1px solid #b9d9ef;
            border-radius: 8px;
            margin-top: 13px;
            padding: 13px 9px 9px 9px;
            font-weight: 600;
            color: #255f86;
        }

        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 6px;
            color: #2f78a7;
            background: #f7fcff;
        }

        QLabel#VolumeInfoLabel,
        QLabel#StatusInfoLabel {
            background: #fbfdff;
            border: 1px solid #c4def0;
            border-radius: 8px;
            padding: 8px;
            color: #2d5d7d;
        }

        QLabel#XrayCaption {
            color: #2c5f82;
            padding-left: 8px;
            font-weight: 600;
        }

        QPushButton {
            background: #d9efff;
            border: 1px solid #a8d3f0;
            border-radius: 8px;
            color: #164465;
            padding: 7px 10px;
            min-height: 24px;
        }

        QPushButton:hover {
            background: #cae9ff;
            border-color: #83c3ea;
        }

        QPushButton:pressed {
            background: #afd9f3;
            border-color: #6aafd9;
        }

        QPushButton:checked {
            background: #68b7e6;
            border-color: #3f96cb;
            color: #ffffff;
        }

        QPushButton:disabled {
            background: #e7eef5;
            border-color: #d2e0ea;
            color: #8ba3b5;
        }

        QLineEdit,
        QComboBox,
        QDoubleSpinBox {
            background: #ffffff;
            border: 1px solid #b8d8ee;
            border-radius: 8px;
            padding: 5px 8px;
            min-height: 24px;
            selection-background-color: #86c9ef;
        }

        QLineEdit:focus,
        QComboBox:focus,
        QDoubleSpinBox:focus {
            border-color: #5aaee5;
            background: #ffffff;
        }

        QComboBox::drop-down,
        QDoubleSpinBox::up-button,
        QDoubleSpinBox::down-button {
            border: 0;
            width: 22px;
        }

        QListWidget {
            background: #fbfdff;
            border: 1px solid #bdddf5;
            border-radius: 8px;
            padding: 4px;
            outline: 0;
        }

        QListWidget::item {
            border-radius: 7px;
            padding: 6px;
            margin: 2px;
        }

        QListWidget::item:hover {
            background: #e4f3ff;
        }

        QListWidget::item:selected {
            background: #bfe3fa;
            color: #113a59;
        }

        QTabWidget::pane {
            background: #fbfdff;
            border: 1px solid #b9d9ef;
            border-radius: 8px;
            top: -1px;
        }

        QTabBar::tab {
            background: #e5f4ff;
            border: 1px solid #b9d9ef;
            border-top-left-radius: 8px;
            border-top-right-radius: 8px;
            padding: 7px 12px;
            margin-right: 2px;
            color: #2b648a;
        }

        QTabBar::tab:selected {
            background: #ffffff;
            color: #1c6898;
            border-bottom-color: #ffffff;
        }

        QSlider::groove:horizontal {
            background: #d9edf9;
            border-radius: 3px;
            height: 6px;
        }

        QSlider::handle:horizontal {
            background: #55aee5;
            border: 2px solid #ffffff;
            border-radius: 8px;
            width: 16px;
            margin: -6px 0;
        }

        QStatusBar {
            background: #dff1ff;
            border-top: 1px solid #c2def1;
            color: #2c5f82;
        }

        QProgressDialog {
            background: #f7fcff;
        }

        QMessageBox {
            background: #f7fcff;
        }
    )"));
}

}  // namespace

int main(int argc, char* argv[])
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication app(argc, argv);
    QApplication::setStyle("Fusion");
    applySoftBlueTheme(app);

    QCoreApplication::setApplicationName("MPR 计划验证");
    QCoreApplication::setApplicationVersion("1.0");

    QCommandLineParser parser;
    parser.setApplicationDescription("交互式 MPR 计划验证程序");
    parser.addHelpOption();
    parser.addVersionOption();

    parser.process(app);

    measurement_app::MprPlanVerificationWindow window;
    window.show();
    return QApplication::exec();
}
