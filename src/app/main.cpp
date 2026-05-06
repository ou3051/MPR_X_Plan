#include "MprPlanVerificationWindow.h"

#include <QApplication>
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

    parser.process(app);

    measurement_app::MprPlanVerificationWindow window;
    window.show();
    return QApplication::exec();
}
