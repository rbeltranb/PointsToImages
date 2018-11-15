# PointsToImages

Given a noisy 3D point cloud model of a real scene, the goal is to extract the planar surfaces such as walls and ground plane. In order to improve the quality or identify elements of interest, each plane extracted is transformed into an image and postprocessed as such, using techniques for image segmentation, edges detection and contour extraction. Although the present work is not focused in the walls reconstruction, the proposed algorithm offers useful results to a later realization of such task. For this algorithm the main goal was centered in the ground plane.


## Plane Segmetation

After a point cloud pre-processing stage with the aim of reduce the data noise and a cloud downsampling to ensure less calculations, the strategy for plane extraction is based on the iteration over the RANSAC variables threshold distance *d* and deviation angle *ε*.

With help of PCL, the estimation of plane equations for walls and ground plane was performed following the next steps:

1. Define the plane with normal *z* as reference plane.
2. Estimate all possible planes per coordinate separately:
   - 2.1. Iteratively, calculate the corresponding planes: first in x-, after in y- and after in z-coordinate using the given *d* and *ε*.
   - 2.2. For each successfully set of inliers, extract it from the point cloud.
   - 2.3. Continue the iterations until the remaining points reach the 40% of the original point cloud
size.
3. Based on a cube assumption, take the outer planes per each coordinate.

As result, all the extracted planes are disposed in separate files for a further analysis. Depending on the values taken for *d* and *ε* are obtained different quantity of planes and with different quality, but in average besides the floor 2 major walls and 2 minor ones are well detected. The ranges used for the variables were: *60 ≤ d ≤ 150 mm* and *4º ≤ ε < 20º* with steps of *10 mm* and *3º* respectively.

With the prospect planes selected, that is a set of six point clouds representing the boundaries of a cubic room, it is possible to start the planes improvement.


## Bitmap Construction

Every detected plane is moved to a common place (*n=\[0,0,1\]*) in order to develop there all the same calculations. The needed plane rotations and translations are performed in such way to match the normal of a given plane with *n*.

To transform each segmented point cloud into a manageable bitmap, all points in the cloud were projected orthogonally over the estimated plane with help of the plane equation.

Normally, the next step would apply a gradient thresholding to get a binarized image, but here it is not necessary due to the fact that the point cloud does not include color information; all the relevant information comes out in white dots on the bitmap, leaving in black the absence of data. To construct with these points a region geometrically manipulable in the bitmap, the closing of the image is applied. For that, a rectangular structuring element was selected, corresponding with the final shape required. The size of this element depending upon the input from the GUI and can change in multiple tests.


## Contour and Image Moments

With the bitmap it is possible obtain the moment of the image and so the area and centroid of it, measures necessary to carry out comparisons between different techniques and parameterizations in the test stage. At the same time, it is feasible the extraction of the closed contours in the image and to apply on them the next series of calculations:

− Weighting by area or length, to discard those contours below a minimum threshold.
− Shape approximations, to measure and compare different polygon classes, e.g. straight and rotated rectangles, concave hulls, smooth regions.
− Membership of pixels, to determine if a random generated pixel lies on a polygon.


## Bitmap Conversion to Point Cloud

As the bitmap is locate over the plane XY and the image centroid corresponds with the coordinate *(0,0,0)*, moving it to R^3 to convert any given silhouette or polygon on a bitmap to its representation as a point cloud, it is enough to project all the pixels perpendicularly from the plane; that is, assigning a value for the *z*-coordinate within an interval defined by a depth. But since not all available pixels can be transformed, it is required to establish a density to describe the permitted number of points per unit area. To reach this, a little algorithm selects the pixel (i.e. coordinate *\[x,y\]*) and performs the calculation of the *z* value making use of the Normal Distribution.


## Implementation

The GUI was developed in Qt and is divided in two windows, one wraps the implementation for plane extraction based on RANSAC algorithm and captures the threshold parameters *d* and *ε*; additionally, this GUI saves the corresponding plane equation, width, height and center of mass in a .xml file next to the segmented point cloud (which remains in a *.pcd* file per plane detected).

The second viewer shows the graphical results for the image improvement and point cloud reconstruction, implementing a viewport based on OpenGL. There are radio buttons for the different algorithms introduced and their corresponding parameters which can be tuned in order to allow multiple combination tests. The check box named 'Fill Polygons' is included to change the viewing mode between point cloud and surface. One menu option exist also to load *.pcd* files to make possible test a single segmented plane.

Two main classes in charge of all operations exist, one for the point cloud manipulation and plane extraction called PlaneDetection, and the other one designed for the geometrical transformations and image processing called Plane3DTransforms. In PlaneDetection is involved the previous implementation and is divided in three parts: one preamble to load the point cloud and preprocessing it by down-sampling and noise removal; one preparation stage, where the input parameters are received and the required algorithms instantiated as objects; and finally, the recursive part where the extraction of walls and ground planes are carry out using RANSAC.

The main task of the actual work is described by the class Plane3DTransforms, although the coordination between pass of parameters, algorithms calling and applying of recursions is provided by two methods in the class QtGLViewer, which can be seen as the *main()* function in a regular C++ implementation.
