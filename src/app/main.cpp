#include <QApplication>
#include <QLabel>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    QLabel label("Measurement MPR planning prototype");
    label.resize(480, 120);
    label.show();
    return QApplication::exec();
}
