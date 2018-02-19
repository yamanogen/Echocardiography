// DICOMread.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
//windows
//#include <stdlib.h>

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
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>

//stream include
#include <iostream>
#include <fstream>

//others
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

using namespace cv;
using namespace std;


#define thresh 0
#define threshH 140
#define threshM 120

#define TimeDelay 300

#define Z1 145.0
#define Z2 170.0
#define X1 20.0
#define X2 60.0

#define Z3 90.0
#define Y1 30.0
#define Y2 60.0

#define D 4.0
#define OutlierTh 0.05f

//global var
int user_data;
int rows, cols, levs, frames;
double PixelSpacingX, PixelSpacingY, PixelSpacingZ;

//frame 10 ok
char DCMfilename1[256] = "C:\\Users\\gen\\Desktop\\心臓データ\\Echocardiography\\qlab\\1\\A_11Jul2017_1.2.840.113663.1500.1.327662638.3.1.20170711.180346.3752_.dcm";
//frame 21
char DCMfilename2[256] = "C:\\Users\\gen\\Desktop\\心臓データ\\echocardio\\x7-250deg60degCenter\\A_11Sep2017_1.2.840.113663.1500.1.327662638.3.1.20170911.190600.671_.dcm";
//frame 9
char DCMfilename3[256] = "C:\\Users\\gen\\Desktop\\心臓データ\\echocardio\\x7-2_72deg100deg_Center\\A_11Sep2017_1.2.840.113663.1500.1.327662638.3.1.20170911.191345.843_.dcm";
//frame 13
char DCMfilename4[256] = "C:\\Users\\gen\\Desktop\\心臓データ\\echocardio\\x3-1_48deg50deg_Center\\A_11Sep2017_1.2.840.113663.1500.1.327662638.3.1.20170911.193525.859_2.dcm";
//frame 20 ok
char DCMfilenameOther[256] = "C:\\Users\\gen\\Desktop\\心臓データ\\sinecho\\other\\A_15Sep2017_1.2.840.113663.1500.1.327662638.3.1.20170915.222916.953_.dcm";
//frame 12
char DCMfilenameGoal1[256] = "C:\\Users\\gen\\Desktop\\心臓データ\\sinecho\\goal\\3dCartesian\\1\\A_15Sep2017_1.2.840.113663.1500.1.327662638.3.1.20170915.221218.234_.dcm";
//frame 12
char DCMfilenameGoal2[256] = "C:\\Users\\gen\\Desktop\\心臓データ\\sinecho\\goal\\3dCartesian\\2\\A_15Sep2017_1.2.840.113663.1500.1.327662638.3.2.20170915.221807.859_.dcm";
//frame 12
char DCMfilenameGoal3[256] = "C:\\Users\\gen\\Desktop\\心臓データ\\sinecho\\goal\\3dCartesian\\3\\A_15Sep2017_1.2.840.113663.1500.1.327662638.3.3.20170915.222310.46_.dcm";
//frame 12
char DCMfilenameFail1[256] = "C:\\Users\\gen\\Desktop\\心臓データ\\sinecho\\fail\\3dCartesian\\1\\A_15Sep2017_1.2.840.113663.1500.1.327662638.3.1.20170915.214725.421_.dcm";
//frame 12
char DCMfilenameFail2[256] = "C:\\Users\\gen\\Desktop\\心臓データ\\sinecho\\fail\\3dCartesian\\2\\A_15Sep2017_1.2.840.113663.1500.1.327662638.3.2.20170915.214755.890_.dcm";
//frame 12
char DCMfilenameFail3[256] = "C:\\Users\\gen\\Desktop\\心臓データ\\sinecho\\fail\\3dCartesian\\3\\A_15Sep2017_1.2.840.113663.1500.1.327662638.3.3.20170915.214831.765_.dcm";
//frame 12
char DCMfilenameFail4[256] = "C:\\Users\\gen\\Desktop\\心臓データ\\sinecho\\fail\\3dCartesian\\4\\A_15Sep2017_1.2.840.113663.1500.1.327662638.3.4.20170915.214845.125_.dcm";


void GetValueFromDCM(DcmDataset *dataset)
{
	DcmElement *elemRows, *elemCols, *elemLevs, *elemPixelSpacingX, *elemPixelSpacingY, *elemPixelSpacingZ, *elemFrames;
	Uint16 Rows, Cols;
	Uint32 Levs;
	Float64 pixelSpacingX, pixelSpacingY, pixelSpacingZ;
	char *Frames;

	if (!dataset->findAndGetElement(DcmTagKey(0x0028, 0x0010), elemRows).good()) {
		printf("Rows couldn't be aquired.\n");
	}
	elemRows->getUint16(Rows);

	if (!dataset->findAndGetElement(DcmTagKey(0x0028, 0x0011), elemCols).good()) {
		printf("Cols couldn't be aquired.\n");
	}
	elemCols->getUint16(Cols);

	if (!dataset->findAndGetElement(DcmTagKey(0x3001, 0x1001), elemLevs).good()) {
		printf("Levs couldn't be aquired.\n");
	}
	elemLevs->getUint32(Levs);

	if (!dataset->findAndGetElement(DcmTagKey(0x0018, 0x602c), elemPixelSpacingX).good()) {
		printf("PixelSpacingX couldn't be aquired.\n");
	}
	elemPixelSpacingX->getFloat64(pixelSpacingX);

	if (!dataset->findAndGetElement(DcmTagKey(0x0018, 0x602e), elemPixelSpacingY).good()) {
		printf("PixelSpacingY couldn't be aquired.\n");
	}
	elemPixelSpacingY->getFloat64(pixelSpacingY);

	if (!dataset->findAndGetElement(DcmTagKey(0x3001, 0x1003), elemPixelSpacingZ).good()) {
		printf("PixelSpacingZ couldn't be aquired.\n");
	}
	elemPixelSpacingZ->getFloat64(pixelSpacingZ);

	if (!dataset->findAndGetElement(DcmTagKey(0x0028, 0x0008), elemFrames).good()) {
		printf("Frames couldn't be aquired.\n");
	}
	elemFrames->getString(Frames);

	rows = (int)Rows;
	cols = (int)Cols;
	levs = (int)Levs;
	PixelSpacingX = (double)pixelSpacingX;
	PixelSpacingY = (double)pixelSpacingY;
	PixelSpacingZ = (double)pixelSpacingZ;
	frames = atoi(Frames);
}

void cloudMake(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Uint8 *pixeldata, int n)
{
	cloud->width = cols;
	cloud->height = rows;
	cloud->is_dense = true;
	cloud->points.resize(cloud->width*cloud->height);

	for (int Z = 0; Z < levs; Z++) {
		Mat frame = Mat(Size(cols, rows), CV_8UC1, pixeldata + (n - 1)*cols*rows*levs + cols*rows*Z);

		for (int X = 0; X < frame.cols; X++) {
			for (int Y = 0; Y < frame.rows; Y++) {
				int I = frame.at<unsigned char>(Y, X);
				
				if (I > thresh) {

					pcl::PointXYZRGB p;
					p.x = X;
					p.y = Y;
					p.z = Z;
					p.r = I;
					p.g = I;
					p.b = I;
					
					if (I > threshM &&Z>45 && Z<135) {
					p.r = I;
					p.g = I;
					p.b = I;
					}
					else {
					p.r = I;
					p.g = I;
					p.b = I;
					}

					if (Z > Z1 && Z < Z2) {
					if (X > 0 && X < ((X2 - X1) / (Z2 - Z1)*(Z - Z1) + 20)) {
					p.r = I;
					p.g = I;
					p.b = I;
					}
					}

					if (Z > Z3 && Z < Z2) {
					if (Y > 0 && Y < ((Y2 - Y1) / (Z2 - Z3)*(Z - Z3) + 30)) {
					p.r = I;
					p.g = I;
					p.b = I;
					}
					}
					
					cloud->push_back(p);
				}
			}
		}
	}
}

void cloudMakeLeft(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Uint8 *pixeldata, int n)
{
	cloud->width = cols;
	cloud->height = rows;
	cloud->is_dense = true;
	cloud->points.resize(cloud->width*cloud->height);
	int count = 0;

	for (int Z = 0; Z < levs; Z++) {
		Mat frame = Mat(Size(cols, rows), CV_8UC1, pixeldata + (n - 1)*cols*rows*levs + cols*rows*Z);

		for (int Y = 0; Y < frame.rows; Y++) {
			count = 0;
			for (int X = 0; X < frame.cols; X++) {
				int I = frame.at<unsigned char>(Y, X);
				if (I > 0 && count < 10 && Z < 170) {
					pcl::PointXYZRGB p;
					p.x = PixelSpacingX*X;
					p.y = PixelSpacingY*Y;
					p.z = PixelSpacingZ*Z;
					p.r = I;
					p.g = I;
					p.b = I;
					cloud->push_back(p);

					count++;
				}
			}
		}
	}
}

void cloudMakeRight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Uint8 *pixeldata, int n)
{
	cloud->width = cols;
	cloud->height = rows;
	cloud->is_dense = true;
	cloud->points.resize(cloud->width*cloud->height);
	int count = 0;

	for (int Z = 0; Z < levs; Z++) {
		Mat frame = Mat(Size(cols, rows), CV_8UC1, pixeldata + (n - 1)*cols*rows*levs + cols*rows*Z);

		for (int Y = 0; Y < frame.rows; Y++) {
			count = 0;
			for (int X = 0; X < frame.cols; X++) {
				int I = frame.at<unsigned char>(Y, cols-1-X);
				if (I > 0 && count < 10 && Z < 170) {
					pcl::PointXYZRGB p;
					p.x = PixelSpacingX*(cols-1-X);
					p.y = PixelSpacingY*Y;
					p.z = PixelSpacingZ*Z;
					p.r = I;
					p.g = I;
					p.b = I;
					cloud->push_back(p);

					count++;
				}
			}
		}
	}
}

void getSideAspect(Uint8 *pixeldata, int n)
{
	int count_l = 0, count_r = 0;
	Mat leftAspect = Mat::zeros(Size(rows, levs), CV_8UC1);
	Mat rightAspect = Mat::zeros(Size(rows, levs), CV_8UC1);
	for (int Z = 0; Z < levs; Z++) {
		Mat frame = Mat(Size(cols, rows), CV_8UC1, pixeldata + (n - 1)*cols*rows*levs + Z*cols*rows);

		for (int Y = 0; Y < frame.rows; Y++) {
			count_l = 0; count_r = 0;
			for (int X = 0; X < frame.cols; X++) {
				int Il = frame.at<unsigned char>(Y, X);
				int Ir = frame.at<unsigned char>(Y, (cols - 1) - X);
				if (Il > 0 && count_l < 10 && Z < 175) {
					if (Il > leftAspect.at<unsigned char>(Z, Y)) {
						leftAspect.at<unsigned char>(Z, Y) = Il;
					}
					count_l++;
				}
				if (Ir > 0 && count_r < 10 && Z < 175) {
					if (Ir > rightAspect.at<unsigned char>(Z, Y)) {
						rightAspect.at<unsigned char>(Z, Y) = Ir;
					}
					count_r++;
				}
			}
		}
		/*
		namedWindow("DICOMtoPNG", WINDOW_AUTOSIZE);
		imshow("DICOMtoPNG", frame);
		waitKey(0);
		destroyAllWindows();
		*/
	}
	namedWindow("LeftAspect", WINDOW_AUTOSIZE);
	imshow("LeftAspect", leftAspect);
	namedWindow("RightAspect", WINDOW_AUTOSIZE);
	imshow("RightAspect", rightAspect);
	waitKey(0);
	destroyAllWindows();

}

void DownSampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered, double filterSizeX, double filterSizeY, double filterSizeZ)
{
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(filterSizeX, filterSizeY, filterSizeZ);
	sor.filter(*cloud_filtered);

	std::cerr << "cloud_filtered: " << cloud_filtered->size() << std::endl;
}

void OutlierException(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered, int K, double T)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(K);
	sor.setStddevMulThresh(T);
	sor.filter(*cloud_filtered);

	std::cerr << "cloud_filtered: " << cloud_filtered->size() << std::endl;
}

void PThrough(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	// Build a passthrough filter to remove spurious NaNs
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(4.3, 11);
	pass.filter(*cloud);
}

void Segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	// All the objects needed
	pcl::PCDWriter writer;
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());

	// Datasets
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

	std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

	// Build a passthrough filter to remove spurious NaNs
	
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, 1.5);
	pass.filter(*cloud_filtered);
	std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_filtered);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	// Create the segmentation object for the planar model and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight(0.1);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.03);
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(cloud_normals);
	// Obtain the plane inliers and coefficients
	seg.segment(*inliers_plane, *coefficients_plane);
	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	// Extract the planar inliers from the input cloud
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);

	// Write the planar inliers to disk
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
	extract.filter(*cloud_plane);
	std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;
	writer.write("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

	// Remove the planar inliers, extract the rest
	extract.setNegative(true);
	extract.filter(*cloud_filtered2);
	extract_normals.setNegative(true);
	extract_normals.setInputCloud(cloud_normals);
	extract_normals.setIndices(inliers_plane);
	extract_normals.filter(*cloud_normals2);

	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(0.1);
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(0.05);
	seg.setRadiusLimits(0, 0.1);
	seg.setInputCloud(cloud_filtered2);
	seg.setInputNormals(cloud_normals2);

	// Obtain the cylinder inliers and coefficients
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

	// Write the cylinder inliers to disk
	extract.setInputCloud(cloud_filtered2);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZRGB>());
	extract.filter(*cloud_cylinder);
	if (cloud_cylinder->points.empty())
		std::cerr << "Can't find the cylindrical component." << std::endl;
	else
	{
		std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl;
		writer.write("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
	}
}

void segmentedColorChange(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients)
{
	if (inliers->indices.size() == 0) {
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
	}
	else {
		std::cerr << "Model coefficients: " << coefficients->values[0] << " "
			<< coefficients->values[1] << " "
			<< coefficients->values[2] << " "
			<< coefficients->values[3] << std::endl;

		std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
		for (size_t i = 0; i < inliers->indices.size(); ++i) {
			//std::cerr << inliers->indices[i] << "    " << cloud1->points[inliers->indices[i]].x << " " << cloud1->points[inliers->indices[i]].y << " " << cloud1->points[inliers->indices[i]].z << std::endl;
			cloud->points[inliers->indices[i]].r = 255;
			cloud->points[inliers->indices[i]].g = 0;
			cloud->points[inliers->indices[i]].b = 0;
		}
	}
}

void projectPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	// Create a set of planar coefficients with X=Y=0,Z=1 or X=Z=0,Y=1 or Y=Z=0,X=1 or others
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[1] = coefficients->values[2] = 0;
	coefficients->values[0] = 1.0;
	coefficients->values[3] = 0;

	// Create the filtering object
	pcl::ProjectInliers<pcl::PointXYZRGB> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud);

}

void compressCoordinate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	for (size_t i = 0; i < cloud->size(); ++i) {
		
		if (((cloud->points[i].x / D) - (int)(cloud->points[i].x / D)) < 0.5) {
			cloud->points[i].x = (int)(cloud->points[i].x / D);
		}
		else {
			cloud->points[i].x = ((int)(cloud->points[i].x / D)) + 1;
		}

		if (((cloud->points[i].y / D) - (int)(cloud->points[i].y / D)) < 0.5) {
			cloud->points[i].y = (int)(cloud->points[i].y / D);
		}
		else {
			cloud->points[i].y = ((int)(cloud->points[i].y / D)) + 1;
		}

		if (((cloud->points[i].z / D) - (int)(cloud->points[i].z / D)) < 0.5) {
			cloud->points[i].z = (int)(cloud->points[i].z / D);
		}
		else {
			cloud->points[i].z = ((int)(cloud->points[i].z / D)) + 1;
		}
		

		/*
		cloud->points[i].x *= PixelSpacingX;
		cloud->points[i].y *= PixelSpacingY;
		cloud->points[i].z *= PixelSpacingZ;
		*/
		//std::cerr << i << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;

	}
	//std::cerr << "cloud1: " << cloud->size() << std::endl;
}

void modifyActualCoordinate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	for (size_t i = 0; i < cloud->size(); ++i) {
		
		cloud->points[i].x *= PixelSpacingX;
		cloud->points[i].y *= PixelSpacingY;
		cloud->points[i].z *= PixelSpacingZ;
		
		//std::cerr << i << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;

	}
}

void outputCSV(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int n)
{
	std::cerr << "start" << std::endl;
	int PixelData[(int)(208 / D) + 1][(int)(112 / D) + 1][(int)(176 / D) + 1];
	int Z, Y, X;
	Z = (int)(208 / D) + 1; Y = (int)(112 / D) + 1; X = (int)(176 / D) + 1;
	for (int z = 0; z < Z; z++) {
		for (int y = 0; y < Y; y++) {
			for (int x = 0; x < X; x++) {
				PixelData[z][y][x] = 0;
				//printf("syoki\n");
			}
		}
	}

	int a, b, c = 0;
	for (size_t i = 0; i < cloud->size(); ++i) {
		a = cloud->points[i].x;
		b = cloud->points[i].y;
		c = cloud->points[i].z;
		if (PixelData[c][b][a] < cloud->points[i].r) {
			PixelData[c][b][a] = cloud->points[i].r;
			
			//printf("%d\n", cloud->points[i].r);
			//printf("done\n");
		}
	}

	char filename[256];
	for (int k = 0; k < Z; k++) {
		sprintf_s(filename, "C:\\Users\\gen\\Desktop\\心臓データ\\Echocardiography\\qlab\\1\\csv3\\%d_%d.csv", n, k + 1);
		ofstream save(filename);
		for (int j = 0; j < Y; j++) {
			for (int i = 0; i < X; i++) {
				char buf[4];
				snprintf(buf, 3, "%d", PixelData[k][j][i]);
				save << buf;
				if(i!=43) save << ',';
			}
			save << endl;
		}
		
		printf("complete:%d_%d\n", n, k + 1);
	}
}

int main()
{
	OFLog::configure(OFLogger::INFO_LOG_LEVEL);

	//point cloud 宣言
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud4(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud5(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud6(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud7(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud8(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud9(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud10(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud11(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud12(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud13(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud14(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud15(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudL(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudR(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	//DICOMからピクセルデータ取得
	DcmFileFormat dcmfile;
	DcmDataset *dataset;
	DcmElement *element;
	Uint8 *pixeldata;
	if (dcmfile.loadFile(DCMfilename1).bad()) {
		printf("DcmFileFormat:loadFile() failed.\n");
		return 1;
	}
	dataset = dcmfile.getDataset();

	if (!dataset->findAndGetElement(DcmTagKey(0x7fe0, 0x0010), element).good()) {
		printf("frames couldn't be aquired.\n");
		return -1;
	}
	GetValueFromDCM(dataset);

	DcmOtherByteOtherWord *pixdata = (DcmOtherByteOtherWord*)element;
	pixdata->getUint8Array(pixeldata);

	//ピクセルデータからcloudへ
	cloudMake(cloud1, pixeldata, 1);
	std::cerr << "cloud1: " << cloud1->size() << std::endl;
	//printCloudNumbers(cloud1);
	/*
	cloudMake(cloud2, pixeldata, 2);
	cloudMake(cloud3, pixeldata, 3);
	cloudMake(cloud4, pixeldata, 4);
	cloudMake(cloud5, pixeldata, 5);
	cloudMake(cloud6, pixeldata, 6);
	cloudMake(cloud7, pixeldata, 7);
	cloudMake(cloud8, pixeldata, 8);
	cloudMake(cloud9, pixeldata, 9);
	
	cloudMake(cloud10, pixeldata, 10);
	cloudMake(cloud11, pixeldata, 11);
	cloudMake(cloud12, pixeldata, 12);
	*/
	//cloudMake(cloud13, pixeldata);
	//cloudMake(cloud14, pixeldata);
	//cloudMake(cloud15, pixeldata);

	//cloudMakeLeft(cloudL, pixeldata, 1);
	//cloudMakeRight(cloudR, pixeldata, 1);
	
	
	//cloudデータ処理
	//getSideAspect(pixeldata, 1);


	//範囲指定除去
	/*
	PThrough(cloud1);
	PThrough(cloud2);
	PThrough(cloud3);
	PThrough(cloud4);
	PThrough(cloud5);
	PThrough(cloud6);
	PThrough(cloud7);
	PThrough(cloud8);
	PThrough(cloud9);
	PThrough(cloud10);
	*/

	
	//外れ値処理
	
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	//OutlierException(cloud1, cloud1, 50, OutlierTh);
	//std::cerr << "cloud1: " << cloud1->size() << std::endl;
	/*
	OutlierException(cloud2, cloud2, 50, OutlierTh);
	OutlierException(cloud3, cloud3, 50, OutlierTh);
	OutlierException(cloud4, cloud4, 50, OutlierTh);
	OutlierException(cloud5, cloud5, 50, OutlierTh);
	OutlierException(cloud6, cloud6, 50, OutlierTh);
	OutlierException(cloud7, cloud7, 50, OutlierTh);
	OutlierException(cloud8, cloud8, 50, OutlierTh);
	OutlierException(cloud9, cloud9, 50, OutlierTh);
	OutlierException(cloud10, cloud10, 50, OutlierTh);
	*/

	//ダウンサンプリング
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

	DownSampling(cloud1, cloud1, D, D, D);

	/*
	DownSampling(cloud2, cloud2, filterSizeX, filterSizeY, filterSizeZ);
	DownSampling(cloud3, cloud3, filterSizeX, filterSizeY, filterSizeZ);
	DownSampling(cloud4, cloud4, filterSizeX, filterSizeY, filterSizeZ);
	DownSampling(cloud5, cloud5, filterSizeX, filterSizeY, filterSizeZ);
	DownSampling(cloud6, cloud6, filterSizeX, filterSizeY, filterSizeZ);
	DownSampling(cloud7, cloud7, filterSizeX, filterSizeY, filterSizeZ);
	DownSampling(cloud8, cloud8, filterSizeX, filterSizeY, filterSizeZ);
	DownSampling(cloud9, cloud9, filterSizeX, filterSizeY, filterSizeZ);
	DownSampling(cloud10, cloud10, filterSizeX, filterSizeY, filterSizeZ);
	*/
	
	//セグメンテーション
	//Segmentation(cloud1);

	//投影
	//projectPlane(cloudL);
	//projectPlane(cloudR);

	//座標修正
	printf("before\n");
	compressCoordinate(cloud1);
	
	/*
	modifyCloudCartesian(cloud2);
	modifyCloudCartesian(cloud3);
	modifyCloudCartesian(cloud4);
	modifyCloudCartesian(cloud5);
	modifyCloudCartesian(cloud6);
	modifyCloudCartesian(cloud7);
	modifyCloudCartesian(cloud8);
	modifyCloudCartesian(cloud9);
	modifyCloudCartesian(cloud10);
	*/
	/*
	OutlierException(cloud1, cloud1, 50, OutlierTh);
	OutlierException(cloud2, cloud2, 50, OutlierTh);
	OutlierException(cloud3, cloud3, 50, OutlierTh);
	OutlierException(cloud4, cloud4, 50, OutlierTh);
	OutlierException(cloud5, cloud5, 50, OutlierTh);
	OutlierException(cloud6, cloud6, 50, OutlierTh);
	OutlierException(cloud7, cloud7, 50, OutlierTh);
	OutlierException(cloud8, cloud8, 50, OutlierTh);
	OutlierException(cloud9, cloud9, 50, OutlierTh);
	OutlierException(cloud10, cloud10, 50, OutlierTh);
	*/

	//CSV出力
	//outputCSV(cloud1, 1);

	/*
	outputCSV(cloud2, 2);
	outputCSV(cloud3, 3);
	outputCSV(cloud4, 4);
	outputCSV(cloud5, 5);
	outputCSV(cloud6, 6);
	outputCSV(cloud7, 7);
	outputCSV(cloud8, 8);
	outputCSV(cloud9, 9);
	outputCSV(cloud10, 10);
	printf("complete all\n");
	*/
	

	//座標展開
	modifyActualCoordinate(cloud1);

	//4Dデータ表示
	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	while (!viewer.wasStopped()) {
		/*
		viewer.showCloud(cloud1);
		printf("cloud1\n");
		Sleep(500);
		viewer.showCloud(cloud1_filtered);
		printf("cloud1_filtered\n");
		Sleep(500);
		*/


		
		//viewer.showCloud(cloudL);
		
		viewer.showCloud(cloud1);
		printf("1\n");
		
		Sleep(TimeDelay);


		/*
		viewer.showCloud(cloud2);
		printf("2\n");
		Sleep(TimeDelay);
		viewer.showCloud(cloud3);
		printf("3\n");
		Sleep(TimeDelay);
		viewer.showCloud(cloud4);
		printf("4\n");
		Sleep(TimeDelay);
		viewer.showCloud(cloud5);
		printf("5\n");
		Sleep(TimeDelay);
		viewer.showCloud(cloud6);
		printf("6\n");
		Sleep(TimeDelay);
		viewer.showCloud(cloud7);
		printf("7\n");
		Sleep(TimeDelay);
		viewer.showCloud(cloud8);
		printf("8\n");
		Sleep(TimeDelay);
		viewer.showCloud(cloud9);
		printf("9\n");
		Sleep(TimeDelay);
		viewer.showCloud(cloud10);
		printf("10\n");
		Sleep(TimeDelay);
		*/

		//拡張
		/*
		viewer.showCloud(cloud7);
		Sleep(TimeDelay);
		printf("8\n");
		viewer.showCloud(cloud8);
		Sleep(TimeDelay);
		printf("9\n");
		viewer.showCloud(cloud9);
		Sleep(TimeDelay);
		printf("10\n");
		*/
		//収縮
		/*
		viewer.showCloud(cloud2);
		Sleep(TimeDelay);
		printf("2\n");
		viewer.showCloud(cloud3);
		Sleep(TimeDelay);
		printf("3\n");
		viewer.showCloud(cloud4);
		Sleep(TimeDelay);
		printf("4\n");
		*/

		//user_data++;
	}


	return 0;
}