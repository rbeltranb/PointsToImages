#ifndef GLVIEWWINDOW_H
#define GLVIEWWINDOW_H

#include <QMainWindow>

#include "qtglviewer.h"

#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>


using namespace std;

namespace Ui
{
class MainWindow;
}


/**
 * @brief The GLViewWindow class
 * @author - Rb2 2018
 */
class GLViewWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit GLViewWindow(QWidget *parent = 0);
    ~GLViewWindow();

    QtGLViewer *getGLViewer();

private slots:
    void on_dilationS_slider_valueChanged(int value);
    void on_thresholdA_slider_valueChanged(int value);
    void on_enclose_Button_pressed();
    void on_planeToCloud_Button_pressed();
    void on_fillRect_chkBox_stateChanged(int isChange);
    void on_loadCloud_Action_triggered();

private:
    Ui::MainWindow *_ui;
    int _dilationSize;
    int _thresholdArea;

};

#endif // GLVIEWWINDOW_H
