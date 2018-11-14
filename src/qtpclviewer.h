#ifndef QTPCLVIEWER_H
#define QTPCLVIEWER_H

#include <QMainWindow>

#include "planedetection.h"
#include "qtglviewer.h"

#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>

using namespace std;

namespace Ui
{
class PCLViewer;
}


/**
 * @brief The QtPCLViewer class
 * @author - Rb2 2018
 */
class QtPCLViewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit QtPCLViewer(QWidget *parent = 0);
    ~QtPCLViewer();

public Q_SLOTS:
    void loadButtonPressed();
    void planeSegmentButtonPressed();

    void thresSliderValChanged(int value);
    void angleSliderValChanged(int value);
    void pSliderValueChanged(int value);

    void resetCloudPointsColor();

    void setGLViewer(QtGLViewer *refGLViewer);

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud;

    unsigned int _segThreshold;
    unsigned int _segAngle;

private:
    Ui::PCLViewer *_ui;
    PlaneDetection _planeDetect;
    QtGLViewer *_refGLViewer;

};

#endif // QTPCLVIEWER_H
