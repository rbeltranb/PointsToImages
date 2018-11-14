#ifndef PLANE3DTRANSFORMS_H
#define PLANE3DTRANSFORMS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "opencv2/opencv.hpp"

using namespace std;


/**
 * @brief The PlaneCharacts struct - Conglomerated measures of a plane.
 */
struct PlaneCharacts {
    float minX=0.0, minY=0.0, minZ=0.0, maxX=0.0, maxY=0.0, maxZ=0.0;
    int width = 0, height = 0, depth = 0;
    float massC[3] = {0.0, 0.0, 0.0};
    int totalPoints = 0;
};


/**
 * @brief The Plane3D2DTransforms class
 * @author - Rb2 2018
 */
class Plane3DTransforms
{
public:
    const static int CONTOUR = 108, BOUND_RECT = 104, ROTD_RECT = 102, CONCAVE_HULL = 101;

    vector< vector<cv::Point3f> > createBoundingRectangle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr,
                                                          int dilationSize, int enclosing, vector<cv::Point3f> floorPoly);
    void filterVectorPoints2D(vector< vector<cv::Point3f> > &vectorPoints3D, bool outerRect);
    void getRotTraslMat(float n0[], float plane[], pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr,
                        cv::Mat &R, cv::Mat &t);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr createPointCloud(vector<cv::Point3f> vectorPointsPoly,
                                                            cv::Scalar color, int depth, int density);
    template <typename UsualVecPoint>
    PlaneCharacts getPlaneCharacteristics(UsualVecPoint vectorPointsPoly);
    cv::Point3f calcIntersection(int planeIds[], vector< std::pair<cv::Mat, int> > vectorPlaneCoeffs);

    /**
     * @brief rotateTraslate - Method to rotate and translate a set the points, which can come from PCL or OPenCV.
     * @param R - Rotation matrix 3x3.
     * @param T - Translation 3-vector.
     * @param vectorRectPoints
     */
    template <typename UsualVecPoint>
    inline void rotateTraslate(cv::Mat R, cv::Mat T, UsualVecPoint &vectorPointsPoly) {
        for (int i = 0; i < vectorPointsPoly.size(); i++) {
            float p0[] = {vectorPointsPoly[i].x, vectorPointsPoly[i].y, vectorPointsPoly[i].z};
            cv::Mat point(1, 3, CV_32FC1, p0);

            point = R*(point.t());
            point = point + T;

            vectorPointsPoly[i].x = point.at<float>(0);
            vectorPointsPoly[i].y = point.at<float>(1);
            vectorPointsPoly[i].z = point.at<float>(2);
        }
    }

private:
    const static int MIN_BOX_AREA = 10000;
    cv::Point2f pointProjection(cv::Point2f p, vector<cv::Point2f> polygon2D);

    inline vector<cv::Point2f> points3Dto2D(vector<cv::Point3f> points3D, int translX, int translY) {
        vector<cv::Point2f> points2D;
        for (int i = 0; i < points3D.size(); i++) {
            points2D.push_back(cv::Point2f(points3D[i].x+translX, points3D[i].y+translY));
        }
        return points2D;
    }
    inline vector<cv::Point3f> points2Dto3D(vector<cv::Point> points2D, int translX, int translY, int posZ) {
        vector<cv::Point3f> points3D;
        for (int i = 0; i < points2D.size(); i++) {
            points3D.push_back(cv::Point3f(points2D[i].x+translX, points2D[i].y+translY, posZ));
        }
        return points3D;
    }

};

#endif // PLANE3DTRANSFORMS_H
