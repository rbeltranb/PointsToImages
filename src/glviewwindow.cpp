#include "glviewwindow.h"
#include "ui_glviewwindow.h"

#include "plane3dtransforms.h"


/**
 * @brief GLViewWindow::GLViewWindow
 * @param parent
 */
GLViewWindow::GLViewWindow(QWidget *parent) :
    QMainWindow(parent),
    _ui(new Ui::MainWindow)
{
    _ui->setupUi(this);
    _dilationSize = 30;
    _thresholdArea = 30;
}


/**
 * @brief GLViewWindow::getGLViewer
 * @return
 */
QtGLViewer *GLViewWindow::getGLViewer() {
    return _ui->openGLWidget;
}


/**
 * @brief GLViewWindow::~GLViewWindow
 */
GLViewWindow::~GLViewWindow() {
    delete _ui;
}


/**
 * @brief GLViewWindow::on_dilationS_slider_valueChanged
 * @param value
 */
void GLViewWindow::on_dilationS_slider_valueChanged(int value) {
    _dilationSize = value;
}


/**
 * @brief GLViewWindow::on_thresholdA_slider_valueChanged
 * @param value
 */
void GLViewWindow::on_thresholdA_slider_valueChanged(int value){
    _thresholdArea = value;
}


/**
 * @brief GLViewWindow::on_enclose_Button_pressed
 */
void GLViewWindow::on_enclose_Button_pressed() {
    _ui->openGLWidget->enclosePolygon(_dilationSize, _ui->fillRect_chkBox->isChecked(),
                                      _ui->contour_radBut->isChecked()?Plane3DTransforms::CONTOUR:
                                      _ui->boundRect_radBut->isChecked()?Plane3DTransforms::BOUND_RECT:
                                      _ui->rotdRect_radBut->isChecked()?Plane3DTransforms::ROTD_RECT:
                                                                        Plane3DTransforms::CONCAVE_HULL,
                                      _ui->outerBox_chkBox->isChecked(), _ui->analytic_chkBox->isChecked());
}


/**
 * @brief GLViewWindow::on_planeToCloud_Button_pressed
 */
void GLViewWindow::on_planeToCloud_Button_pressed() {
    _ui->openGLWidget->planesToCloud(_thresholdArea);
}


/**
 * @brief GLViewWindow::on_fillRect_ckBox_stateChanged
 * @param isChange
 */
void GLViewWindow::on_fillRect_chkBox_stateChanged(int change) {
    Q_UNUSED(change);
    _ui->openGLWidget->accionChangedFillRect();
}


/**
 * @brief GLViewWindow::on_loadCloud_Action_triggered
 */
void GLViewWindow::on_loadCloud_Action_triggered() {
    QString filename = QFileDialog::getOpenFileName(
                this, tr("Open Plane Cloud Point"), QDir::currentPath(),
                tr("PCD files (*.pcd) ;; All files (*.*)"), 0, QFileDialog::DontUseNativeDialog);

    if( !filename.isNull() ) {
        if (filename.endsWith("pcd")) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::io::loadPCDFile(filename.toStdString(), *cloud);
            vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vectorCloudPlanes = {cloud};

            int numPlane = filename.section('_',-1,-1).section('.',-2,-2).toInt();

            cv::Mat planeEqn;
            cv::FileStorage fileC(filename.toStdString()+"_eqn.xml", cv::FileStorage::READ);
            fileC["plane_eqn"] >> planeEqn; fileC.release();

            if (!planeEqn.empty()) {
                vector< pair<cv::Mat, int> > vectorPlaneCoeffs = {make_pair(planeEqn, numPlane)};
                _ui->openGLWidget->setVectorCloudPlanes(vectorCloudPlanes, vectorPlaneCoeffs, 30);
            } else {
                QMessageBox::warning(this, tr("File not found"), tr("XML with plane coefficients file is required."));
            }
        }
    }
}
