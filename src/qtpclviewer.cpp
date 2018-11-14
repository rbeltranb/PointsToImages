#include "qtpclviewer.h"
#include "ui_qtpclviewer.h"

#include <QFileDialog>
#include <pcl/io/pcd_io.h>


/**
 * @brief QtPCLViewer::PCLQtViewer
 * @param parent
 */
QtPCLViewer::QtPCLViewer(QWidget *parent) :
    QMainWindow(parent),
    _ui(new Ui::PCLViewer)
{
    _ui->setupUi(this);
    this->setWindowTitle("Qt - PCL Viewer");

    // The default values
    _segThreshold = 60;
    _segAngle = 4;

    // Setup the cloud pointer
    _cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    // The number of points in the cloud
    _cloud->points.resize(200);

    // Fill the cloud with some points
    for (size_t i = 0; i < _cloud->points.size(); ++i) {
        _cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        _cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        _cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);

        _cloud->points[i].r = _cloud->points[i].g = _cloud->points[i].b = 128;
    }

    // Set up the QVTK window
    _viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    _ui->qvtkWidget->SetRenderWindow(_viewer->getRenderWindow());
    _viewer->setupInteractor(_ui->qvtkWidget->GetInteractor(), _ui->qvtkWidget->GetRenderWindow());
    _ui->qvtkWidget->update();

    // Connect buttons and the function
    connect(_ui->pushButton_load, SIGNAL(clicked()), this, SLOT(loadButtonPressed()));
    connect(_ui->pushButton_planeSegment, SIGNAL(clicked()), this, SLOT(planeSegmentButtonPressed()));

    // Connect sliders and their functions
    connect(_ui->horizontalSlider_G, SIGNAL(valueChanged(int)), this, SLOT(thresSliderValChanged(int)));
    connect(_ui->horizontalSlider_B, SIGNAL(valueChanged(int)), this, SLOT(angleSliderValChanged(int)));
    connect(_ui->horizontalSlider_G, SIGNAL(sliderReleased()), this, SLOT(resetCloudPointsColor()));
    connect(_ui->horizontalSlider_B, SIGNAL(sliderReleased()), this, SLOT(resetCloudPointsColor()));

    // Connect point size slider
    connect(_ui->horizontalSlider_p, SIGNAL(valueChanged(int)), this, SLOT(pSliderValueChanged(int)));

    _viewer->addPointCloud(_cloud, "cloud");
    pSliderValueChanged(2);
    _viewer->resetCamera();
    _ui->qvtkWidget->update();
}


/**
 * @brief QtPCLViewer::planeSegmentButtonPressed
 */
void QtPCLViewer::planeSegmentButtonPressed() {
    _viewer->removeAllPointClouds();

    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vectorCloudPlanes;
    vector< pair<cv::Mat, int> > vectorPlaneCoeffs;
    _planeDetect.startSegmentation(_segThreshold, _segAngle, vectorCloudPlanes, vectorPlaneCoeffs, _cloud);

    int i = 0;
    for (auto it = vectorCloudPlanes.begin(); it != vectorCloudPlanes.end(); ++it) {
        _viewer->addPointCloud<pcl::PointXYZRGB>(*it, "cloud_plane_"+to_string(i++));
    }

    // Transfering of the clouds' planes and its equations to GL Viewer.
    _refGLViewer->setVectorCloudPlanes(vectorCloudPlanes, vectorPlaneCoeffs, _segThreshold);

    _refGLViewer->repaint();
    _ui->qvtkWidget->update();
}


/**
 * @brief QtPCLViewer::loadButtonPressed
 */
void QtPCLViewer::loadButtonPressed() {
    QString filename = QFileDialog::getOpenFileName(
                this, tr("Open Cloud Point File"), QDir::currentPath(),
                tr("PCD files (*.pcd) ;; XML files (*.xml) ;; All files (*.*)"), 0, QFileDialog::DontUseNativeDialog);

    if( !filename.isNull() ) {
        _viewer->removeAllPointClouds();

        if (filename.endsWith("pcd")) {
            _cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::io::loadPCDFile(filename.toStdString(), *_cloud);
        } else if (filename.endsWith("xml")) {
            cv::FileStorage fileC(filename.toStdString(), cv::FileStorage::READ);
            cv::Mat points;
            fileC["point_cloud"] >> points;
            fileC.release();
        }

        resetCloudPointsColor();
        _viewer->addPointCloud(_cloud, "cloud");
        _ui->qvtkWidget->update();

        if (_ui->planeExtract->isChecked()) {
            _cloud = _planeDetect.prepareSegmentation(_cloud);
        }
    }
}


/**
 * @brief QtPCLViewer::sliderReleased
 */
void QtPCLViewer::resetCloudPointsColor() {
    // Set the default color
    for (size_t i = 0; i < _cloud->size(); i++) {
        _cloud->points[i].r = _cloud->points[i].g = _cloud->points[i].b = 128;
    }
    _viewer->updatePointCloud(_cloud, "cloud");
    _ui->qvtkWidget->update();
}


/**
 * @brief QtPCLViewer::ttSliderValueChanged
 * @param value
 */
void QtPCLViewer::thresSliderValChanged(int value) {
    _segThreshold = value;
}


/**
 * @brief QtPCLViewer::csSliderValueChanged
 * @param value
 */
void QtPCLViewer::angleSliderValChanged(int value) {
    _segAngle = value;
}


/**
 * @brief QtPCLViewer::pSliderValueChanged
 * @param value
 */
void QtPCLViewer::pSliderValueChanged(int value) {
    _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
    _ui->qvtkWidget->update();
}


/**
 * @brief QtPCLViewer::setGLViewer
 * @param refGLViewer - Reference used to pass the plane parameters into the GL Viewer.
 */
void QtPCLViewer::setGLViewer(QtGLViewer *refGLViewer) {
    _refGLViewer = refGLViewer;
}


/**
 * @brief QtPCLViewer::~PCLQtViewer
 */
QtPCLViewer::~QtPCLViewer() {
    delete _ui;
}
