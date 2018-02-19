//#include "stdafx.h"

#include "EcocardiographyDataPath.h"

//windows
//#include <stdlib.h>
#include <direct.h>
#include <math.h>
#include <algorithm>


//DCMTK include
#include <dcmtk/config/osconfig.h>
#include <dcmtk/dcmdata/dctk.h>
#include <dcmtk/dcmimgle/dcmimage.h>
#include <dcmtk/dcmimage/diregist.h>

//OpenCV include
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

//pcl include
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/pfh.h>
#include <pcl/features/ppf.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/bilateral.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/cg/correspondence_grouping.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/correspondence.h>
#include <pcl/pcl_base.h>
#include <pcl/ModelCoefficients.h>


//stream include
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <sstream>

//others
#include <vtkAutoInit.h>

//define

//PixelTh goal fail target pitch roll 30
//PixelTh mani 80
#define PixelTh 15



#define OutlierTh 0.1f
#define OutlierK 20
#define D 2.0
#define DofKeypoints 4.0
#define times 1.0
#define timesV 0.5
#define div1 4
#define div2 8
//divをそれぞれのクラスタリングで決定できるようソースコード変更
//ベクトルの検出精度向上のための３次元特徴量選出

////Goal, Fail
//#define b1 177
//#define b2 175
//#define b3 170
//#define b4 210

////Test1
//#define b1 180
//#define b2 177
//#define b3 180
//#define b4 210

////Mani
//#define b1 177
//#define b2 180
//#define b3 180
//#define b4 210

//BIG
#define b1 160
#define b2 190
#define b3 170
#define b4 190

#define R 185
//
////Goal, Fail
//#define delta_x -5
//#define delta_y 15//1/7*rows
//#define delta_z 6

////Mani
//#define delta_x 0
//#define delta_y 15//1/7*rows
//#define delta_z 8

//BIG
#define delta_x 0
#define delta_y 0//1/7*rows
#define delta_z 8

#define TimeDelay 1000

#define border 0.05

//global var
extern int rows, cols, levs, frames;
extern double PixelSpacingX, PixelSpacingY, PixelSpacingZ;

using namespace cv;
using namespace std;
using namespace pcl;


void GetValueFromDCM(DcmDataset *dataset);
void constructPointXYZRGB(PointCloud<PointXYZRGB>::Ptr &cloud);
void constructPointXYZRGBNormal(PointCloud<PointXYZRGBNormal>::Ptr &cloud);
void constructNormal(PointCloud<Normal>::Ptr &cloud_normals);
void constructPointNormal(PointCloud<PointNormal>::Ptr &cloud_PointNormal);
void createPointNormal(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointNormal>::Ptr &cloud_PointNormal);
void constructSHOT352(PointCloud<SHOT352>::Ptr &cloud_descriptors);
void constructPFH(PointCloud<PFHSignature125>::Ptr &cloud_descriptors);

void cloudMake(PointCloud<PointXYZRGB>::Ptr cloud, Uint8 *pixeldata, int n);
void cloudPixelThRemoval(PointCloud<PointXYZRGB>::Ptr &cloud, int Th);
void normalsEstimation(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr cloud_normals);
void PointNormalEstimation(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointNormal>::Ptr cloud_PointNormal);
void createKeypoints(PointCloud<PointXYZRGB>::Ptr cloud, vector<float> &curvature_v, PointCloud<PointXYZRGB>::Ptr cloud_keypoints);
void fastCreateKeypoints(PointCloud<PointXYZRGB>::Ptr cloud, vector<float> &curvature_v, PointCloud<PointXYZRGB>::Ptr cloud_keypoints);

void cloudMakeLeft(PointCloud<PointXYZRGB>::Ptr cloud, Uint8 *pixeldata, int n);
void cloudMakeRight(PointCloud<PointXYZRGB>::Ptr cloud, Uint8 *pixeldata, int n);
void getSideAspect(Uint8 *pixeldata, int n);


void uniformSampling(PointCloud<PointXYZRGB>::Ptr cloud, double leafSize, PointCloud<PointXYZRGB>::Ptr cloud_filtered);
void DownSampling(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr cloud_filtered, double filterSizeX, double filterSizeY, double filterSizeZ);
void statisticalOutlierRemoval(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr cloud_filtered, int K, double T);
void PThrough(PointCloud<PointXYZRGB>::Ptr cloud);
void Segmentation(PointCloud<PointXYZRGB>::Ptr cloud);
void segmentedColorChange(PointCloud<PointXYZRGB>::Ptr cloud, PointIndices::Ptr inliers, ModelCoefficients::Ptr coefficients);
void projectPlane(PointCloud<PointXYZRGB>::Ptr cloud);

void compressCoordinate(PointCloud<PointXYZRGB>::Ptr cloud);
void modifyCloudCoordinate(PointCloud<PointXYZRGB>::Ptr cloud);
void modifyCloudACoordinate(PointCloud<PointXYZRGBA>::Ptr cloudA);
void modifyNormalCloudCoordinate(PointCloud<PointNormal>::Ptr cloud_PointNormal);
void modifyPointNormalCoordinate(PointCloud<PointXYZRGBNormal>::Ptr cloud);
void modifyVectorCoordinate(PointCloud<PointNormal>::Ptr vector);
void outputCSV(PointCloud<PointXYZRGB>::Ptr cloud, int n);
void vectorMake(PointCloud<PointNormal>::Ptr vector, int F);
void printVector(PointCloud<PointNormal>::Ptr vector);
void addVectorLine(PointCloud<PointNormal>::Ptr vector, boost::shared_ptr<visualization::PCLVisualizer> viewer);
void addCloud(PointCloud<PointXYZRGB>::Ptr cloud, boost::shared_ptr<visualization::PCLVisualizer> viewer);
void removeVectorLine(PointCloud<PointNormal>::Ptr vector, boost::shared_ptr<visualization::PCLVisualizer> viewer);
PointCloud<PointNormal>::Ptr Surface_normals(PointCloud<PointXYZRGB>::Ptr cloud);
PointCloud<PointWithScale> Extract_SIFT(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointNormal>::Ptr cloud_normals);
void setColorForEachCurvature(PointCloud<PointXYZRGB>::Ptr cloud, vector<float> curvature_v);
void distributionMapCurvatures(vector<float> curvature_v);


void calculateSHOTOMP(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr cloud_normals, PointCloud<PointXYZRGB>::Ptr cloud_keypoints, PointCloud<SHOT352>::Ptr cloud_descriptors);
void calculatePFH(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr cloud_normals, PointCloud<PFHSignature125>::Ptr cloud_descriptors, float radius);
void calculatePPF(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr cloud_normals, PointCloud<PPFSignature>::Ptr cloud_descriptors);
void matchDescriptors(PointCloud<PointXYZRGB>::Ptr cloud1_keypoints, PointCloud<PointXYZRGB>::Ptr cloud2_keypoints, PointCloud<SHOT352>::Ptr cloud1_descriptors, PointCloud<SHOT352>::Ptr cloud2_descriptors, vector<Correspondence> &cloud_corrs);
void matchDescriptors2(PointCloud<PointXYZRGB>::Ptr cloud1_keypoints, PointCloud<PointXYZRGB>::Ptr cloud2_keypoints, PointCloud<PFHSignature125>::Ptr cloud1_descriptors, PointCloud<PFHSignature125>::Ptr cloud2_descriptors, vector<Correspondence> &cloud_corrs);

void reversibleDescriptorMatching(vector<Correspondence> &cloud_corrs_forward, vector<Correspondence> &cloud_corrs_reverse, vector<Correspondence> &cloud_corrs);
void vectorOutCSV(vector<Correspondence> &cloud_corrs_current, PointCloud<PointXYZRGB>::Ptr cloud_current, PointCloud<PointXYZRGB>::Ptr cloud_next, int current);

void convertHistgram(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr cloud_filtered);
void Binarization(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr cloud_filtered);
void Threshold(PointCloud<PointXYZRGB>::Ptr cloud);
void edgeDetection(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr cloud_normals, vector<PointIndices> &label_indices);
void splitCloudfromColor(PointCloud<PointXYZRGB>::Ptr cloud1_keypoints, PointCloud<PointXYZRGB>::Ptr cloud2_keypoints, vector<Correspondence> &cloud_corrs, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_colored);
void CSVout_numberColoredCloud(vector<vector<PointCloud<PointXYZRGB>::Ptr>> cloud_colored);
void CSVoutVector(PointCloud<PointXYZRGB>::Ptr cloud1_keypoints, PointCloud<PointXYZRGB>::Ptr cloud2_keypoints, vector<Correspondence> &cloud_corrs,int frame);
void CSVoutVectorAVE(PointXYZRGB vector_ave, int frame);
void splitCloudintoCloudSmallGrid(PointCloud<PointXYZRGB>::Ptr cloud, vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>> &cloud_smallgrid, PointCloud<PointXYZRGB>::Ptr cloud1_keypoints, PointCloud<PointXYZRGB>::Ptr cloud2_keypoints, vector<Correspondence> &cloud_corrs, vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>> &cloud_smallgrid_vector);
void splitCloudintoCloudSmallGrid2(PointCloud<PointXYZRGB>::Ptr cloud, vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>> &cloud_smallgrid, PointCloud<PointXYZRGB>::Ptr cloud1_keypoints, PointCloud<PointXYZRGB>::Ptr cloud2_keypoints, vector<Correspondence> &cloud_corrs, vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>> &cloud_smallgrid_vector);
void splitCloudintoCloudSmallGridFastDebug(PointCloud<PointXYZRGB>::Ptr cloud, vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>> &cloud_smallgrid);
void clusteringBloodRegion(vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_smallgrid_onlypoints);
void clusteringBloodRegion2(vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>>> &cloud_smallgrid_onlypoints_blood, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_smallgrid_onlypoints);
void clusteringBloodRegion3(vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>>> &cloud_smallgrid_onlypoints_blood, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_smallgrid_onlypoints, int count);
void clusteringBloodRegionFastDebug(vector<vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>>> &cloud_smallgrid, vector<vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>>> &cloud_smallgrid_onlypoints_blood, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_smallgrid_onlypoints, int count);
void clusteringBloodRegion4(vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_smallgrid_onlypoints);
void clusteringHighPixel(vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>>> &cloud_smallgrid_onlypoints_highpixel, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_smallgrid_onlypoints);
void clusteringAtrialSeptum(vector<vector<int>> &timing, vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_smallgrid_onlypoints);
void clusteringVentricularSeptum(vector<vector<int>> &timing, vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_smallgrid_onlypoints);
void clusteringLeftVentricle(vector<vector<int>> &timing, vector<PointCloud<PointXYZRGB>::Ptr> cloud_blood_clustered, vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_Ventricle);
void clusteringLeftVentricle2(vector<vector<int>> &timing, vector<PointCloud<PointXYZRGB>::Ptr> cloud_clustered_Ventricle, vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_Ventricle);
void clusteringLeftAtrium(vector<vector<int>> &timing, vector<PointCloud<PointXYZRGB>::Ptr> cloud_blood_clustered, vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_Atrium);
void clusteringLeftAtrium2(vector<vector<int>> &timing, vector<PointCloud<PointXYZRGB>::Ptr> cloud_blood_clustered, vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_Atrium);
void clusteringAorta(vector<vector<int>> &timing, vector<PointCloud<PointXYZRGB>::Ptr> cloud_blood_clustered, vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_Aorta);
void clusteringAorta2(vector<vector<int>> &timing, vector<PointCloud<PointXYZRGB>::Ptr> cloud_blood_clustered, vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_Aorta);
void displayGridInfo(visualization::PCLVisualizer *viewer, vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>> &cloud_smallgrid_vector, int k, int j, int i);
void euclideanClustering(PointCloud<PointXYZRGB>::Ptr cloud, float Tolerance, int size_min, int size_max, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_clustered);
void recognitionBloodRegion(vector<PointCloud<PointXYZRGB>::Ptr> &cloud_clustered_before, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_clustered_after);
void recognitionBloodRegion2(vector<PointCloud<PointXYZRGB>::Ptr> &cloud_clustered_before, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_clustered_after);
void bilateralFiltering(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr cloud_filtered);
void outerProductFromNormal(PointCloud<PointNormal>::Ptr cloud_PointNormal, int X, int Y, int Z, PointCloud<PointXYZRGB>::Ptr &cloud_info, PointCloud<Normal>::Ptr &cloud_Normal_info, PointCloud<PointNormal>::Ptr &cloud_PointNormal_ave_info);
void locationEstimation_valve(vector<vector<int>> &timing, vector<PointCloud<PointXYZRGB>::Ptr> cloud_blood_clustered, vector<PointCloud<PointXYZRGB>::Ptr> cloud_Ventricle, vector<PointCloud<PointXYZRGB>::Ptr> cloud_Atrium, vector<PointCloud<PointXYZRGB>::Ptr> cloud_Aorta, vector<PointCloud<PointXYZRGB>::Ptr> cloud, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_MitralValve, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_AorticValve);
void locationEstimation_valve2(vector<vector<int>> &timing, vector<PointCloud<PointXYZRGB>::Ptr> cloud_blood_clustered, vector<PointCloud<PointXYZRGB>::Ptr> cloud, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_MitralValve, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_AorticValve);
void CSVoutMitralValveCentroid(PointCloud<PointXYZRGB>::Ptr Mitral_Valve, int filenumber, int fileindex);
void CSVoutAorticValveCentroid(PointCloud<PointXYZRGB>::Ptr Aortic_Valve, int filenumber, int fileindex);
void averagePointCloud(vector<PointCloud<PointXYZRGB>::Ptr> cloud, PointCloud<PointXYZRGBA>::Ptr &cloud_ave);

void oneSlideView(visualization::PCLVisualizer *viewer, vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>> &cloud_smallgrid, int i, int j, int k);
void createOneSlice(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr &cloud_filtered, int i, int j, int k, int theta);
