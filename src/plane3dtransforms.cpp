#include "plane3dtransforms.h"

#include <pcl/filters/extract_indices.h>


/**
 * @brief Plane3DTransforms::createBoundingRectangle - This method carry out three important steps: 1. Converts the
 * point cloud of a given plane in a bitmap, 2. Determines the image centroid and performs an additional translation
 * to that centroid, 3. Extracts the contour and applies the rule for the boundig calculation.
 * @param pointCloudPtr
 * @param dilationSize - Major axis for the structuring element.
 * @param enclosing - Defines if apply looking for Outer/Rotated Rectangles or Polygons.
 * @param floorPoly - Intersection points of the wall planes with the ground area.
 * @return - A single list with all 4 corners of each rectangle.
 */
vector< vector<cv::Point3f> > Plane3DTransforms::createBoundingRectangle(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr, int dilationSize, int enclosing,
        vector<cv::Point3f> floorPoly) {

    PlaneCharacts plnCharacts =
            getPlaneCharacteristics<pcl::PointCloud<pcl::PointXYZRGB>::VectorType>(pointCloudPtr->points);
    plnCharacts.width += dilationSize*2.4;
    plnCharacts.height += dilationSize*2.4;

    /// Uses a 20% extra for morphological operations.
    int translX = abs(plnCharacts.minX) + dilationSize*1.2;
    int translY = abs(plnCharacts.minY) + dilationSize*1.2;
    vector<cv::Point2f> floorPoly2D = points3Dto2D(floorPoly, translX, translY);

    // +1 to compensate the float to int round in the farthest pixel.
    cv::Mat image = cv::Mat::zeros(plnCharacts.height+1/*rows:y*/, plnCharacts.width+1/*cols:x*/, CV_8UC1);
    for (int i = 0; i < pointCloudPtr->points.size(); i++) {
        image.at<uchar>(translY+pointCloudPtr->points.at(i).y, translX+pointCloudPtr->points.at(i).x) = 255;
    }

    cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(2*dilationSize+1, 2*dilationSize+1),
                                            cv::Point(dilationSize, dilationSize));
    cv::Mat dilationDst;
    /// Applies the dilation operation to join the 2D points.
    cv::dilate(image, dilationDst, element);
    /// Applies the erosion operation to form an image.
    cv::erode(dilationDst, image, element);

    vector<vector<cv::Point>> contours;
    cv::findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    vector< vector<cv::Point3f> > vectorPoints3D;

    for (int i = 0; i < contours.size(); i++) {
        vector<cv::Point3f> points3D;

        /// Finds the bounding rectangle that encloses the area of interest.
        if (enclosing == BOUND_RECT) {
            cv::Rect rect = cv::boundingRect( contours[i] );
            points3D.push_back(cv::Point3f(rect.x-translX, rect.y-translY, plnCharacts.massC[2]));
            points3D.push_back(cv::Point3f(rect.x-translX, rect.y+rect.height-translY, plnCharacts.massC[2]));
            points3D.push_back(cv::Point3f(rect.x+rect.width-translX, rect.y+rect.height-translY, plnCharacts.massC[2]));
            points3D.push_back(cv::Point3f(rect.x+rect.width-translX, rect.y-translY, plnCharacts.massC[2]));
        }

        /// Finds the rotated rectangles for each contour.
        else if (enclosing == ROTD_RECT) {
            cv::Point2f pointsRect[4];
            cv::RotatedRect minRect = cv::minAreaRect( cv::Mat(contours[i]) );
            minRect.points(pointsRect);
            points3D = points2Dto3D(vector<cv::Point>(pointsRect, pointsRect+4), -translX, -translY, plnCharacts.massC[2]);
        }

        /// Finds the concave polygons for each contour.
        else if (enclosing == CONCAVE_HULL) {
            vector<cv::Point> pointsPoly;
            float epsilon = cv::arcLength(contours[i], true)/dilationSize;
            cv::approxPolyDP(contours[i], pointsPoly, epsilon, true);
            points3D = points2Dto3D(pointsPoly, -translX, -translY, plnCharacts.massC[2]);
        }

        /// Uses the fully contour.
        else { // enclosing == CONTOUR

            /// The analytic limits are taken into account only for the ground plane.
            if (floorPoly2D.size()) {
                for (int j= 0; j < contours[i].size(); j++) {
                    /// Leaves only those points which belong to the analytic square.
                    if (cv::pointPolygonTest(floorPoly2D, contours[i][j], false) >= 0) {
                        points3D.push_back(cv::Point3f(contours[i][j].x-translX, contours[i][j].y-translY, plnCharacts.massC[2]));
                    } else {
                        /// Calculates the projection point on the closest line of the analytical square.
                        cv::Point2f pointProj = pointProjection(contours[i][j], floorPoly2D);
                        points3D.push_back(cv::Point3f(pointProj.x-translX, pointProj.y-translY, plnCharacts.massC[2]));
                    }
                }
            } else {
                points3D = points2Dto3D(contours[i], -translX, -translY, plnCharacts.massC[2]);
            }
        }

        if (points3D.size()) vectorPoints3D.push_back(points3D);
    }

    return vectorPoints3D;
}


/**
 * @brief Plane3DTransforms::pointProjection - Performs the orthogonal projection of a given point on the closest line
 * of certain polygon.
 * @param p - Point close to the line.
 * @param polygon2D - Set of points that define the polygon.
 * @return - Point on the segment's line.
 */
cv::Point2f Plane3DTransforms::pointProjection(cv::Point2f p, vector<cv::Point2f> polygon2D) {
    cv::Point2f pointProj;

    for (int k = 1; k < polygon2D.size()+1; k++) {
        cv::Point2f a = polygon2D[k-1], b = polygon2D[k%polygon2D.size()];
        cv::Point2f ab = b-a;
        cv::Point2f ap = p-a, q;
        float t = ap.dot(ab) / ab.dot(ab);
        if (t < 0.0) q = a;
        else if (t > 1.0) q = b;
        else q = a + t*ab;

        if (norm(pointProj)==0 || norm(p-q)<norm(p-pointProj)) pointProj = q;
    }

    return pointProj;
}


/**
 * @brief Plane3DTransforms::filterVectorPoints2D - Applies intersection methods to eliminate non necessary rectangles
 * from a given list, taking into account only the plane X-Y. Also creates a outer box if it is required.
 * @param vectorPoints3D
 * @param outerRect
 */
void Plane3DTransforms::filterVectorPoints2D(vector< vector<cv::Point3f> > &vectorPoints3D, bool outerRect) {
    PlaneCharacts figure1, figure2, figure3;
    vector< vector<cv::Point3f> > innerVectors;

    /// Builds and exclusion list to keep only valid boxes.
    for (int i = 0; i < vectorPoints3D.size(); i++) {
        figure1 = getPlaneCharacteristics(vectorPoints3D[i]);

        /// Includes too small boxes in the exclusion list.
        if (figure1.width*figure1.height > MIN_BOX_AREA)

            for (int j = 0; j < vectorPoints3D.size(); j++) {
                figure2 = getPlaneCharacteristics(vectorPoints3D[j]);

                /// Checks for Fig1's coordinates containment into Fig 2.
                if (figure2.minX < figure1.minX && figure2.minY < figure1.minY &&
                    figure2.maxX > figure1.maxX && figure2.maxY > figure1.maxY) {
                    innerVectors.push_back(vectorPoints3D[i]);
                    break;
                }
            }

        else innerVectors.push_back(vectorPoints3D[i]);
    }

    /// Subtracts exclusion from the original list.
    for (int i = 0; i < innerVectors.size(); i++) {
        vectorPoints3D.erase(find(vectorPoints3D.begin(), vectorPoints3D.end(), innerVectors[i]));
    }

    if (outerRect && !vectorPoints3D.empty()) {
        figure3 = getPlaneCharacteristics(vectorPoints3D[0]);

        /// Locates the global maximum and minimum coordinates.
        for (int i = 1; i < vectorPoints3D.size(); i++) {
            figure1 = getPlaneCharacteristics(vectorPoints3D[i]);

            if (figure1.minX < figure3.minX) figure3.minX = figure1.minX;
            if (figure1.minY < figure3.minY) figure3.minY = figure1.minY;
            if (figure1.maxX > figure3.maxX) figure3.maxX = figure1.maxX;
            if (figure1.maxY > figure3.maxY) figure3.maxY = figure1.maxY;
        }

        /// Creates a bounding box.
        vector<cv::Point3f> points3D;
        points3D.push_back(cv::Point3f(figure3.minX, figure3.minY, figure3.massC[2]));
        points3D.push_back(cv::Point3f(figure3.maxX, figure3.minY, figure3.massC[2]));
        points3D.push_back(cv::Point3f(figure3.maxX, figure3.maxY, figure3.massC[2]));
        points3D.push_back(cv::Point3f(figure3.minX, figure3.maxY, figure3.massC[2]));
        vectorPoints3D.push_back(points3D);
    }
}


/**
 * @brief Plane3DTransforms::getRotTraslMat - Performs the rotation of the plane normal around the reference vector n0.
 * Implements the Euler-Rodriges' formula.
 * @param n0 - Reference vector to place the given plane.
 * @param plane - Plane equation of the plane to be rotated.
 * @param pointCloudPtr - Point cloud of the plane to use its limits.
 * @param R - Resulted rotation matrix.
 * @param T - Resulted translation vector.
 */
void Plane3DTransforms::getRotTraslMat(float n0[], float plane[],
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr, cv::Mat &R, cv::Mat &t) {
    cv::Mat I = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat n = cv::Mat(1, 3, CV_32FC1, n0).t();
    cv::Mat p = cv::Mat(1, 3, CV_32FC1, plane).t();

    /// Euler-Rodriges' formula:
    cv::Mat np = n+p;
    cv::Mat np2 = np.t()*np;
    R = 2*np*(np.t()) / np2.at<float>(0) - I;
    /// Rotation matrix:
    R = R.t();

    /// Extracts the mass center of the plane:
    PlaneCharacts plnCharacts =
            getPlaneCharacteristics<pcl::PointCloud<pcl::PointXYZRGB>::VectorType>(pointCloudPtr->points);
    float t0[] = {plnCharacts.massC[0], plnCharacts.massC[1], plnCharacts.massC[2]};
    /// Translation vector:
    t = cv::Mat(1, 3, CV_32FC1, t0).t(); // t() function clones the Mat!
    // Final Translation to origin point must be -R*t
}


/**
 * @brief Plane3DTransforms::getPlaneCharacteristics - Method used to calculate the moments of a given set of points.
 * @param vectorPointsPoly - A template that covers Point objects from PCL and OpenCV.
 * @return - A structure with the moments of the set.
 */
template <typename UsualVecPoint>
PlaneCharacts Plane3DTransforms::getPlaneCharacteristics(UsualVecPoint vectorPointsPoly) {
    int i = 0;
    PlaneCharacts plnCharacts;
    plnCharacts.minX = vectorPointsPoly[i].x; plnCharacts.maxX = vectorPointsPoly[i].x;
    plnCharacts.minY = vectorPointsPoly[i].y; plnCharacts.maxY = vectorPointsPoly[i].y;
    plnCharacts.minZ = vectorPointsPoly[i].z; plnCharacts.maxZ = vectorPointsPoly[i].z;

    for (i = 1; i < vectorPointsPoly.size(); i++) {
        if (vectorPointsPoly[i].x < plnCharacts.minX) plnCharacts.minX = vectorPointsPoly[i].x;
        else if (vectorPointsPoly[i].x > plnCharacts.maxX) plnCharacts.maxX = vectorPointsPoly[i].x;
        if (vectorPointsPoly[i].y < plnCharacts.minY) plnCharacts.minY = vectorPointsPoly[i].y;
        else if (vectorPointsPoly[i].y > plnCharacts.maxY) plnCharacts.maxY = vectorPointsPoly[i].y;
        if (vectorPointsPoly[i].z < plnCharacts.minZ) plnCharacts.minZ = vectorPointsPoly[i].z;
        else if (vectorPointsPoly[i].z > plnCharacts.maxZ) plnCharacts.maxZ = vectorPointsPoly[i].z;
    }
    plnCharacts.totalPoints = i;

    plnCharacts.width = abs(plnCharacts.maxX - plnCharacts.minX);
    plnCharacts.height = abs(plnCharacts.maxY - plnCharacts.minY);
    plnCharacts.depth = abs(plnCharacts.maxZ - plnCharacts.minZ);

    plnCharacts.massC[0] = plnCharacts.minX + plnCharacts.width/2;
    plnCharacts.massC[1] = plnCharacts.minY + plnCharacts.height/2;
    plnCharacts.massC[2] = plnCharacts.minZ + plnCharacts.depth/2;

    return plnCharacts;
}


/**
 * @brief Plane3DTransforms::createPointCloud - Method to converts a given region to its representation as a point
 * cloud. The algorithm projects random pixels perpendicularly to the plane X-Y with a random z defined into depth.
 * @param vectorPointsPoly - Points 0 and 2 must be in opposite corners.
 * @param color - RGB.
 * @param depth - Defines an interval to assigning a value for z.
 * @param density - Indcates the permitted number of points per unit area.
 * @return
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Plane3DTransforms::createPointCloud(
        vector<cv::Point3f> vectorPointsPoly, cv::Scalar color, int depth, int density) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    PlaneCharacts plnCharacts = getPlaneCharacteristics<vector<cv::Point3f>>(vectorPointsPoly);
    vector<cv::Point2f> pointsContourn = points3Dto2D(vectorPointsPoly, 0, 0);
    pcl::PointIndices::Ptr outliers(new pcl::PointIndices());

    int densityArea = plnCharacts.width*plnCharacts.height * (float)density/MIN_BOX_AREA;
    cloud->points.resize(densityArea);
    cv::RNG rng(densityArea);
    depth /= 2;

    for (int i = 0; i < densityArea; i++) {
        cv::Point2f point(rng.uniform(plnCharacts.minX, plnCharacts.maxX),
                          rng.uniform(plnCharacts.minY, plnCharacts.maxY));

        if (cv::pointPolygonTest(pointsContourn, point, false) > 0) {
            cloud->points[i].x = point.x;
            cloud->points[i].y = point.y;
            cloud->points[i].z = rng.uniform(plnCharacts.massC[2]-depth, plnCharacts.massC[2]+depth);

            cloud->points[i].r = color[0];
            cloud->points[i].g = color[1];
            cloud->points[i].b = color[2];
        } else {
            outliers->indices.push_back(i);
        }
    }

    /// Removes the unused points from the cloud.
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(outliers);
    extract.setNegative(true);
    extract.filter(*cloud);

    return cloud;
}


/**
 * @brief Plane3DTransforms::calcIntersection - Calculates the intersection point given three planes.
 * @param planeIds - IDs selected to be used from the complete plane list.
 * @param vectorPlaneCoeffs - Complete list of plane equations given as an input to the program.
 * @return - 3D intersection point.
 */
cv::Point3f Plane3DTransforms::calcIntersection(int planeIds[], vector< std::pair<cv::Mat, int> > vectorPlaneCoeffs) {

    cv::Mat coeffs = vectorPlaneCoeffs[planeIds[0]].first.clone();
    coeffs.push_back(vectorPlaneCoeffs[planeIds[1]].first);
    coeffs.push_back(vectorPlaneCoeffs[planeIds[2]].first);

    cv::Mat detA = coeffs.colRange(0,3);
    cv::Mat detI = detA.clone(); coeffs.col(3).copyTo(detI.col(0));
    cv::Mat detJ = detA.clone(); coeffs.col(3).copyTo(detJ.col(1));
    cv::Mat detK = detA.clone(); coeffs.col(3).copyTo(detK.col(2));

    cv::Point3f point;
    float det = cv::determinant(detA);
    point.x = cv::determinant(detI) / det;
    point.y = cv::determinant(detJ) / det;
    point.z = cv::determinant(detK) / det;

    // Due to the y-coordinate inverted in OpenGL.
    return -point;
}
