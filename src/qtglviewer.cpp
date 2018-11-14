#include "qtglviewer.h"

#include <iostream>


/**
 * @brief QtGLViewer::QGLBegin
 * @param parent
 */
QtGLViewer::QtGLViewer(QWidget *parent) :
    QOpenGLWidget(parent)
{
    _xRot = _yRot = 0.0;
    _scale = 7;
    _fillRect = false;
    _depth = 45;
}


/**
 * @brief QtGLViewer::initializeGL
 */
void QtGLViewer::initializeGL() {
    glClearColor(0.0, 0.0, 0.0, 0.0);
}


/**
 * @brief QtGLViewer::paintGL
 */
void QtGLViewer::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glRotatef(_xRot, 1.0, 0.0, 0.0);
    glRotatef(_yRot, 0.0, 0.0, 1.0);
    glScalef(_scale, _scale, _scale);

    if (!_fillRect)
    for (auto it = _vectorCloudPlanes.begin(); it != _vectorCloudPlanes.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr = *it;

        glBegin(GL_POINTS);
        for (int i = 0; i < pointCloudPtr->points.size(); i++) {
            glColor3f(pointCloudPtr->points.at(i).r, pointCloudPtr->points.at(i).g, pointCloudPtr->points.at(i).b);
            glVertex3f(pointCloudPtr->points.at(i).x, pointCloudPtr->points.at(i).y, pointCloudPtr->points.at(i).z);
        }
        glEnd();

        glPushAttrib(GL_CURRENT_BIT);
    }

    for (int i = 0; i < _vectorPointsPolys3D.size(); i++) {
        vector< vector<cv::Point3f> > pointsPolys = _vectorPointsPolys3D[i].first;
        cv::Scalar color = _vectorPointsPolys3D[i].second;
        glColor3f(color[0], color[1], color[2]);

        for (int j = 0; j < pointsPolys.size(); j++) {
            if (!_fillRect) glBegin(GL_LINE_STRIP);
            else {glBegin(GL_POLYGON); if (i == 1) continue;} // The ceiling is skipped.
            for (int k = 0; k < pointsPolys[j].size(); k++) {
                glVertex3d(pointsPolys[j][k].x, pointsPolys[j][k].y, pointsPolys[j][k].z);
            }
            glVertex3d(pointsPolys[j][0].x, pointsPolys[j][0].y, pointsPolys[j][0].z);
            glEnd();
        }

        glPushAttrib(GL_CURRENT_BIT);
    }
}


/**
 * @brief QtGLViewer::resizeGL
 * @param width
 * @param height
 */
void QtGLViewer::resizeGL(int width, int height) {
    width *= 100; height *=100;

    int side = qMin(width, height);
    glViewport((width-side)/2, (height-side)/2, side, side);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-width/2, width/2, -height/2, height/2, -width/2, width/2);
    glMatrixMode(GL_MODELVIEW);
}


/**
 * @brief QtGLViewer::mousePressEvent
 * @param event
 */
void QtGLViewer::mousePressEvent(QMouseEvent *event) {
    _lastPos = event->pos();
}


/**
 * @brief QtGLViewer::mouseMoveEvent
 * @param event
 */
void QtGLViewer::mouseMoveEvent(QMouseEvent *event) {
    float dx = event->pos().x() - _lastPos.x();
    float dy = event->pos().y() - _lastPos.y();

    // While the buttons are presssed:
    if(event->buttons() & Qt::LeftButton) {
        _xRot += dy;
        _yRot += dx;
    }

    _lastPos = event->pos();
    update();
}


/**
 * @brief QtGLViewer::wheelEvent
 * @param event
 */
void QtGLViewer::wheelEvent(QWheelEvent *event) {
    _scale -= event->delta() / 1.0e3;
    update();
}


/**
 * @brief QtGLViewer::setVectorCloudPlanes - Function to receive the detected planes from PLC-Viewer.
 * @param vectorCloudPlanes - Collection of detected planes unlabeled.
 * @param vectorPlaneCoeffs - Corresponding equations to the plane list.
 * @param depth - Threshold [distance] used in the plane segmentation.
 */
void QtGLViewer::setVectorCloudPlanes(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vectorCloudPlanes,
                                      vector< pair<cv::Mat, int> > vectorPlaneCoeffs, int depth) {
    _vectorCloudPlanes = vectorCloudPlanes;
    _vectorPlaneCoeffs = vectorPlaneCoeffs;
    _vectorPointsPolys3D.clear();
    _depth = depth;
    update();
}


/**
 * @brief QtGLViewer::enclosePolygon - This method uses a list of six filtered planes which forms an imaginary cube of
 * the scene, calculates the intersections between them, and for each one calculates and applies the Rotation and
 * Translation necessary to extract their boundaries in a common coordinate system.
 * @param dilationSize - Size for the structuring element used in the dilation of the image.
 * @param bool fillRect - Flag to change the showing mode in the GL viewport.
 * @param enclosing - Parameter with the select enclosing method.
 * @param outerBox - Flag to calculate the biggest rectangle which encloses all the point data.
 * @param analytic - Flag to include the planes intersection with analytical methods.
 */
void QtGLViewer::enclosePolygon(int dilationSize, bool fillRect, int enclosing, bool outerBox, bool analytic) {
    _fillRect = fillRect;
    _vectorPointsPolys3D.clear();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr copyCloudPlane(new pcl::PointCloud<pcl::PointXYZRGB>);
    // The Normal Vector Reference takes the z direction from the groung plane.
    float n[] = {0.0, 0.0, _vectorPlaneCoeffs[0].first.at<float>(2)/abs(_vectorPlaneCoeffs[0].first.at<float>(2))};

    int i = 0;
    vector<cv::Point3f> floorPoly;
    if (_vectorCloudPlanes.size() == 6) {
        /// The first plane in the list (i.e. floor) is analytically calculated.
        floorPoly.push_back(_plane3DTransforms.calcIntersection( (int[]){0, 2, 4}, _vectorPlaneCoeffs) );
        floorPoly.push_back(_plane3DTransforms.calcIntersection( (int[]){0, 2, 5}, _vectorPlaneCoeffs) );
        floorPoly.push_back(_plane3DTransforms.calcIntersection( (int[]){0, 3, 5}, _vectorPlaneCoeffs) );
        floorPoly.push_back(_plane3DTransforms.calcIntersection( (int[]){0, 3, 4}, _vectorPlaneCoeffs) );

        if (analytic) {
            vector< vector<cv::Point3f> > pointsPoly; pointsPoly.push_back(floorPoly);
            _vectorPointsPolys3D.push_back(make_pair(pointsPoly, cv::Scalar(_vectorCloudPlanes[0]->points.at(0).r,
                                                                            _vectorCloudPlanes[0]->points.at(0).g,
                                                                            _vectorCloudPlanes[0]->points.at(0).b)));
            i++;
        }
    }

    for (; i < _vectorCloudPlanes.size(); i++) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr = _vectorCloudPlanes[i];
        if (!pointCloudPtr->points.size()) {_vectorPointsPolys3D.push_back(_vectorPointsPolys3D[i-1]); continue;}
        *copyCloudPlane = *pointCloudPtr;

        cv::Mat R, t;
        _plane3DTransforms.getRotTraslMat(n, (float*)_vectorPlaneCoeffs[i].first.data, copyCloudPlane, R, t);

        _plane3DTransforms.rotateTraslate<pcl::PointCloud<pcl::PointXYZRGB>::VectorType>(
                    R, -R*t, copyCloudPlane->points);

        if (i == 0) _plane3DTransforms.rotateTraslate<vector<cv::Point3f>>(R, -R*t, floorPoly); else floorPoly.clear();

        vector< vector<cv::Point3f> > pointsPolys3D = _plane3DTransforms.createBoundingRectangle(
                    copyCloudPlane, dilationSize, enclosing, floorPoly);
        _plane3DTransforms.filterVectorPoints2D(pointsPolys3D, outerBox);

        /// The resultant contour detection is a list of all possible ones found.
        for (int j = 0; j < pointsPolys3D.size(); j++) {
            _plane3DTransforms.rotateTraslate<vector<cv::Point3f>>(R.t(), t, pointsPolys3D[j]);
        }

        _vectorPointsPolys3D.push_back(make_pair(pointsPolys3D, cv::Scalar(pointCloudPtr->points.at(0).r,
                                                                           pointCloudPtr->points.at(0).g,
                                                                           pointCloudPtr->points.at(0).b)));
    }

    update();
}


/**
 * @brief QtGLViewer::planesToCloud - This method takes the previously calculated list, which contains all the detected
 * polygons that enclose regions on all the available planes.
 * @param density - A parameter to stimate the maximun points per unit area in the resultant point cloud.
 */
void QtGLViewer::planesToCloud(int density) {
    // The Normal Vector Reference keeps the original sign.
    float n[] = {0.0, 0.0, _vectorPlaneCoeffs[0].first.at<float>(3)/_vectorPlaneCoeffs[0].first.at<float>(3)};

    for (int i = 0; i < _vectorCloudPlanes.size(); i++) {
        cv::Mat R, t; 
        _plane3DTransforms.getRotTraslMat(n, (float*)_vectorPlaneCoeffs[i].first.data, _vectorCloudPlanes[i], R, t);

        vector< vector<cv::Point3f> > pointsPolys3D = _vectorPointsPolys3D[i].first;

        _vectorCloudPlanes[i]->clear();
        for (int j = 0; j < pointsPolys3D.size(); j++) {
            _plane3DTransforms.rotateTraslate<vector<cv::Point3f>>(R, -R*t, pointsPolys3D[j]);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = _plane3DTransforms.createPointCloud(
                        pointsPolys3D[j], _vectorPointsPolys3D[i].second, _depth, density);

            _vectorCloudPlanes[i]->insert(_vectorCloudPlanes[i]->end(), cloud->begin(), cloud->end());
        }

        _plane3DTransforms.rotateTraslate<pcl::PointCloud<pcl::PointXYZRGB>::VectorType>(
                    R.t(), t, _vectorCloudPlanes[i]->points);

        // The second plane in the list corresponds to the ceiling.
        if (i == 0 && _vectorCloudPlanes.size()>1) {i++; _vectorCloudPlanes[i]->clear();}
    }

    update();
}


/**
 * @brief QtGLViewer::accionChangedFillRect
 */
void QtGLViewer::accionChangedFillRect() {
    _fillRect = !_fillRect;
    update();
}
