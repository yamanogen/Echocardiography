#include "stdafx.h"
#include "MyHeader.h"

VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

int main(void)
{
	OFLog::configure(OFLogger::INFO_LOG_LEVEL);

	//DICOM����s�N�Z���f�[�^�擾
	DcmFileFormat dcmfile;
	DcmDataset *dataset;
	DcmElement *element;
	Uint8 *pixeldata;
	if (dcmfile.loadFile(DCMfilenameGoal1).bad()) {
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

	//point cloud declare and make
	vector<PointCloud<PointXYZRGB>::Ptr> cloud;
	PointCloud<PointXYZRGB>::Ptr cloud_elem;
	for (int i = 0; i < 8; i++) {//more than 1
		constructPointXYZRGB(cloud_elem);
		cloudMake(cloud_elem, pixeldata, i + 1);
		cloud.push_back(cloud_elem);
	}

	//�͈͎w�菜��

	// DownSampling
	for (size_t i = 0; i < cloud.size(); i++) {
		uniformSampling(cloud[i], D, cloud[i]);
	}

	//�O��l����
	for (int n = 0; n < 3; n++) {
		for (size_t i = 0; i < cloud.size(); i++) {
			statisticalOutlierRemoval(cloud[i], cloud[i], OutlierK, OutlierTh);
		}
	}

	//gaussian smoothing
	vector<PointCloud<PointXYZRGB>::Ptr> cloud_smoothed;
	PointCloud<PointXYZRGB>::Ptr cloud_smoothed_elem;
	for (size_t i = 0; i < cloud.size(); i++) {
		constructPointXYZRGB(cloud_smoothed_elem);
		cloud_smoothed.push_back(cloud_smoothed_elem);
	}
	for (size_t i = 0; i < cloud.size(); i++) {
		bilateralFiltering(cloud[i], cloud_smoothed[i]);
	}
	

	// Compute the normals 
	/*vector<PointCloud<Normal>::Ptr> cloud_normals;
	PointCloud<Normal>::Ptr cloud_normals_elem;
	for (size_t i = 0; i < cloud.size(); i++) {
		constructNormal(cloud_normals_elem);
		cloud_normals.push_back(cloud_normals_elem);
	}
	for (size_t i = 0; i < cloud.size(); i++) {
		normalsEstimation(cloud[i], cloud_normals[i]);
	}*/

	//transfer
	for (size_t f = 0; f < cloud.size(); f++) {
		transformPointCloud(*cloud_smoothed[f], *cloud_smoothed[f], Eigen::Vector3f(-200, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
	}

	//���W�W�J
	for (size_t f = 0; f < cloud.size(); f++) {
		modifyCloudCoordinate(cloud[f]);
		modifyCloudCoordinate(cloud_smoothed[f]);
	}

	// Display 4D Data
	visualization::PCLVisualizer *viewer(new visualization::PCLVisualizer("4D Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	int current = 0, next = 1;
	bool show_points_(true);
	bool show_smoothedpoints_(true);

	while (!viewer->wasStopped()) {
		printf("current : %d\n", current);
		viewer->spinOnce(TimeDelay);
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		if (show_points_) {
			viewer->addPointCloud(cloud[current], "cloud");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
		}
		if (show_smoothedpoints_) {
			viewer->addPointCloud(cloud_smoothed[current], "cloud_smoothed");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_smoothed");
		}

		char text[16];
		sprintf_s(text, "current:%d", current);
		viewer->addText(text, 100, 0, "current");

		current = next;
		if (next != (cloud.size() - 1)) {
			next++;
		}
		else {
			next = 0;
		}
	}

	return 0;
}