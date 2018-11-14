#ifndef QTGLVIEWER_H
#define QTGLVIEWER_H

#include <QtWidgets>

#include "plane3dtransforms.h"

using namespace std;


/**
 * @brief The QtGLViewer class
 * @author - Rb2 2018
 */
class QtGLViewer : public QOpenGLWidget
{
    Q_OBJECT
public:
    explicit QtGLViewer(QWidget *parent = 0);
    void setVectorCloudPlanes(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vectorCloudPlanes,
                              vector< pair<cv::Mat, int> > vectorPlaneCoeffs, int depth);
    void enclosePolygon(int dilationSize, bool fillRect, int enclosing, bool outerBox, bool analytic);
    void planesToCloud(int density);
    void accionChangedFillRect();

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

signals:

public slots:

private:
    float _xRot, _yRot, _scale;
    QPoint _lastPos;
    bool _fillRect;
    int _depth;

    Plane3DTransforms _plane3DTransforms;
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> _vectorCloudPlanes;
    vector< std::pair<cv::Mat, int> > _vectorPlaneCoeffs;
    vector< pair<vector< vector<cv::Point3f> >, cv::Scalar> > _vectorPointsPolys3D;

};

#endif // QTGLVIEWER_H
