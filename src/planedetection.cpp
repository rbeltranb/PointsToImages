#include "planedetection.h"
#include "plane3dtransforms.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/extract_indices.h>

#include "opencv2/core/persistence.hpp"


/**
 * @brief PlaneDetection::prepareSegmentation - It is the former main() function.
 * @param cloud
 * @return
 * @author - Sundaresh 2017
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlaneDetection::prepareSegmentation(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::copyPointCloud(*cloud, *cloudrgb);
    vector<int> indices;

    //Remove NaN from the data
    pcl::removeNaNFromPointCloud(*cloudrgb, *cloud_filtered, indices);

    //Save it to other file
    pcl::io::savePCDFileASCII("data/cloud_filtered.pcd", *cloud_filtered);

    //Random sample filter to downsample the data without much loss of information
    pcl::RandomSample<pcl::PointXYZRGB> ran;
    ran.setInputCloud(cloud_filtered);
    int no_points = (int)(.9*cloud_filtered->points.size());
    ran.setSample(no_points);
    ran.filter(*final);

    //Remove the outliers
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    outrem.setInputCloud(final);
    outrem.setRadiusSearch(50);
    outrem.setMinNeighborsInRadius(50);
    // apply filter
    outrem.filter(*final);

    //Save the downsampled data
    pcl::io::savePCDFileASCII("data/final.pcd", *final);

    return final;
}


/**
 * @brief PlaneDetection::startSegmentation
 * @param thresholdDist
 * @param epsilonAngle
 * @param finalvisVector - Collection of detected planes unlabeled.
 * @param vectorPlaneCoeff - Collection of calculated plane equations.
 * @param final
 * @return - True if some error occurs.
 * @author - Sundaresh 2017, Rb2 2018
 */
int PlaneDetection::startSegmentation(int thresholdDist, int epsilonAngle,
                                      vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &finalvisVector,
                                      vector< pair<cv::Mat, int> > &vectorPlaneCoeff,
                                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr final) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_copy(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);

    /**
     * @brief upLowLimits -- [i_low,  mass_center_low ]
     *                       [i_high, mass_center_high]
     */
    float upLowLimits[2][2];
    float massCenterVal;
    Eigen::Vector3f normals[] = {PLANE_XY, PLANE_XZ, PLANE_YZ};

    for (int j = 0; j < 3; j++) {
        vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> selPlanesVector;
        vector< pair<cv::Mat, int> > selPCoeffVector;

        int i = 0;
        //for (int angle = 4; angle < 20; angle+=3) { // Unit tests...
        int angle = epsilonAngle;
        //for (int thrdis = 60; thrdis < 160; thrdis+=10) { // Unit tests...
        int thrdis = thresholdDist;
        *final_copy = *final;
        int nr_points = (int)final_copy->points.size();
        seg.setDistanceThreshold(thrdis);
        seg.setEpsAngle(angle * M_PI / 180);

        // Condition to stop estimation
        while (final_copy->points.size() > 0.35 * nr_points) {

            if (doSegmentation(i, normals[j], final_copy, seg, selPlanesVector, selPCoeffVector, massCenterVal))
                break;

            if (i == 0) {
                upLowLimits[0][0] = upLowLimits[1][0] = i;
                upLowLimits[0][1] = upLowLimits[1][1] = massCenterVal;
            } else if (massCenterVal < upLowLimits[0][1]) {
                upLowLimits[0][0] = i;
                upLowLimits[0][1] =  massCenterVal;
            } else if (massCenterVal > upLowLimits[1][1]) {
                upLowLimits[1][0] = i;
                upLowLimits[1][1] =  massCenterVal;
            }

            i++;
        }
        //}
        //}

        // Extraction of outer planes.
        if (selPlanesVector.size()) {
            finalvisVector.push_back(selPlanesVector[upLowLimits[0][0]]);
            vectorPlaneCoeff.push_back(selPCoeffVector[upLowLimits[0][0]]);
            finalvisVector.push_back(selPlanesVector[upLowLimits[1][0]]);
            vectorPlaneCoeff.push_back(selPCoeffVector[upLowLimits[1][0]]);
        }
    }

    return 0;
}


/**
 * @brief PlaneDetection::doSegmentation - Reusing code to perform iteratively the segmentation.
 * @param i
 * @param planeCoord
 * @param final
 * @param seg
 * @param finalvisVector - Collection of detected planes unlabeled.
 * @param vectorPlaneCoeff - Collection of calculated plane equations.
 * @param massCenterVal
 * @return - True if some error occurs.
 * @author - Sundaresh 2017, Rb2 2018
 */
int PlaneDetection::doSegmentation(int i, Eigen::Vector3f planeCoord, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &final,
                                   pcl::SACSegmentation<pcl::PointXYZRGB> seg,
                                   vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &finalvisVector,
                                   vector< pair<cv::Mat, int> > &vectorPlaneCoeff, float &massCenterVal) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::PCDWriter writer;
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    seg.setInputCloud(final);
    seg.setAxis(planeCoord);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        cerr << "Could not estimate a planar model for the given dataset." << endl;
        return 1;
    }

    // Extract the inliers
    extract.setInputCloud(final);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_p); // cloud_p is holding the inliers extracted by ransac

    Plane3DTransforms plane3DTransforms;
    PlaneCharacts plnCharacts = plane3DTransforms.getPlaneCharacteristics
            <pcl::PointCloud<pcl::PointXYZRGB>::VectorType>(cloud_p->points);

    if (planeCoord == PLANE_XY) {
        for (size_t a = 0; a < cloud_p->size(); ++a) {
            cloud_p->points[a].r = 255;
            cloud_p->points[a].g = 0   + i*25;
            cloud_p->points[a].b = 255 - i*25;
        }
        massCenterVal = plnCharacts.massC[2];
    } else if (planeCoord == PLANE_XZ) {
        for (size_t a = 0; a < cloud_p->size(); ++a) {
            cloud_p->points[a].r = 0;
            cloud_p->points[a].g = 100 + i*25;
            cloud_p->points[a].b = 250 - i*25;
        }
        massCenterVal = plnCharacts.massC[1];
    } else { // PLANE_YZ
        for (size_t a = 0; a < cloud_p->size(); ++a) {
            cloud_p->points[a].r = 100 + i*25;
            cloud_p->points[a].g = 250 - i*25;
            cloud_p->points[a].b = 0;
        }
        massCenterVal = plnCharacts.massC[0];
    }

    finalvisVector.push_back(cloud_p);

    // Create the filtering object
    extract.setNegative(true);
    extract.filter(*cloud_f); // outliers for the next iteration step
    final.swap(cloud_f);

    // write inliers to pcd-file
    stringstream ss;
    ss << "data/" << "plane_per_" << planeCoord.transpose() << "_" << i << ".pcd";
    writer.write <pcl::PointXYZRGB>(ss.str(), *cloud_p, false);

    // Saving data equation as XML:
    float coeffs[] = {coefficients->values[0], coefficients->values[1], coefficients->values[2],
                      coefficients->values[3]};
    cv::Mat planeEqn = cv::Mat(1, 4, CV_32FC1, coeffs).clone();
    cv::FileStorage fileR(ss.str()+"_eqn.xml", cv::FileStorage::WRITE);
    fileR << "plane_eqn" << planeEqn;
    fileR << "width" << plnCharacts.width;
    fileR << "heigth" << plnCharacts.height;
    fileR << "mass_c" << vector<float>(plnCharacts.massC, plnCharacts.massC+3);
    fileR.release();

    vectorPlaneCoeff.push_back(make_pair(planeEqn, i));

    return 0;
}
