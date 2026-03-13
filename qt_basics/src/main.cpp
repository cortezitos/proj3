#include "mouse_tracker_window.h"

#include <QApplication>

int main(int argc, char * argv[])
{
    QApplication app(argc, argv);

    MouseTrackerWindow window;
    window.show();

    return app.exec();
}
