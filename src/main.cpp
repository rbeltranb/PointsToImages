#include <QApplication>
#include <QMainWindow>

#include "qtpclviewer.h"
#include "glviewwindow.h"


/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main (int argc, char *argv[])
{
    QApplication a(argc, argv);

    GLViewWindow glViewWin;
    QtPCLViewer pclViewer;
    pclViewer.setGLViewer(glViewWin.getGLViewer());

    pclViewer.show();
    glViewWin.show();

    return a.exec();
}
