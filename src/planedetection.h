#ifndef PLANEDETECTION_H
#define PLANEDETECTION_H

#include <pcl/segmentation/sac_segmentation.h>
#include "opencv2/core/mat.hpp"

using namespace std;

/**
 * @brief The PlaneDetection class
 * @author - Rb2 2018
 */
class PlaneDetection
{
public:
    const Eigen::Vector3f PLANE_XY = Eigen::Vector3f(0, 0, 1),
                          PLANE_XZ = Eigen::Vector3f(0, 1, 0),
                          PLANE_YZ = Eigen::Vector3f(1, 0, 0);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prepareSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    int startSegmentation(int thresholdDist, int epsilonAngle,
                          vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &finalvisVector,
                          vector< pair<cv::Mat, int> > &vectorPlaneCoeff,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr final);

private:
    int doSegmentation(int i, Eigen::Vector3f planeCoord, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &final,
                       pcl::SACSegmentation<pcl::PointXYZRGB> seg,
                       vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &finalvisVector,
                       vector< pair<cv::Mat, int> > &vectorPlaneCoeff, float &massCenterVal);
};

#endif // PLANEDETECTION_H
