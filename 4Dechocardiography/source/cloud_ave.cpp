#include "stdafx.h"
#include "MyHeader.h"

VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

int main(void)
{
	OFLog::configure(OFLogger::INFO_LOG_LEVEL);

	//DICOMからピクセルデータ取得
	DcmFileFormat dcmfile;
	DcmDataset *dataset;
	DcmElement *element;
	Uint8 *pixeldata;
	if (dcmfile.loadFile(DCMfilenameTarget).bad()) {
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

	//convert Histgram
	for (size_t i = 0; i < cloud.size(); i++) {
		convertHistgram(cloud[i], cloud[i]);
	}

	//範囲指定除去

	// DownSampling
	for (size_t i = 0; i < cloud.size(); i++) {
		uniformSampling(cloud[i], D, cloud[i]);
	}

	PointCloud<PointXYZRGBA>::Ptr cloud_ave(new PointCloud<PointXYZRGBA>);
	averagePointCloud(cloud, cloud_ave);

	////外れ値処理
	//for (int n = 0; n < 2; n++) {
	//	for (size_t i = 0; i < cloud.size(); i++) {
	//		statisticalOutlierRemoval(cloud[i], cloud[i], OutlierK, OutlierTh);
	//	}
	//}

	//gaussian smoothing
	/*vector<PointCloud<PointXYZRGB>::Ptr> cloud_smoothed;
	PointCloud<PointXYZRGB>::Ptr cloud_smoothed_elem;
	for (size_t i = 0; i < cloud.size(); i++) {
	constructPointXYZRGB(cloud_smoothed_elem);
	cloud_smoothed.push_back(cloud_smoothed_elem);
	}
	for (size_t i = 0; i < cloud.size(); i++) {
	bilateralFiltering(cloud[i], cloud_smoothed[i]);
	}*/

	/*for (size_t i = 0; i < cloud.size(); i++) {
		bilateralFiltering(cloud[i], cloud[i]);
	}*/
	

	//座標展開
	for (size_t f = 0; f < cloud.size(); f++) {
		modifyCloudCoordinate(cloud[f]);
	}
	modifyCloudACoordinate(cloud_ave);


	// Display 4D Data
	visualization::PCLVisualizer *viewer(new visualization::PCLVisualizer("4D Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	int current = 0, next = 1;
	bool show_points_(false);
	bool show_ave_(true);

	while (!viewer->wasStopped()) {
		printf("current : %d\n", current);
		viewer->spinOnce(TimeDelay);
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		if (show_points_) {
			viewer->addPointCloud(cloud[current], "cloud");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
		}
		if (show_ave_) {
			viewer->addPointCloud(cloud_ave, "cloud_ave");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_ave");
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