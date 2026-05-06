#include "MprPlanVerificationWindow.h"

#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QSurfaceFormat>

#include <QVTKOpenGLNativeWidget.h>

int main(int argc, char* argv[])
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication app(argc, argv);
    QCoreApplication::setApplicationName("measurement_mpr_interactive_test");
    QCoreApplication::setApplicationVersion("0.1");

    QCommandLineParser parser;
    parser.setApplicationDescription("Interactive MPR test program");
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption dicomFolderOption(
        "dicom-folder",
        "Load a CT DICOM folder on startup.",
        "path");
    parser.addOption(dicomFolderOption);
    parser.process(app);

    measurement_app::MprPlanVerificationWindow window(parser.value(dicomFolderOption));
    window.show();
    return QApplication::exec();
}
