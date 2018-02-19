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
	if (dcmfile.loadFile(DCMfilenameMani1).bad()) {
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
	for (int i = 0; i < 8; i++) {
		constructPointXYZRGB(cloud_elem);
		cloudMake(cloud_elem, pixeldata, i + 1);
		cloud.push_back(cloud_elem);
	}

	for (size_t i = 0; i < cloud.size(); i++) {
		convertHistgram(cloud[i], cloud[i]);
	}

	for (size_t i = 0; i < cloud.size(); i++) {
		cloudPixelThRemoval(cloud[i], 30);
	}

	//外れ値処理
	for (int n = 0; n < 2; n++) {
		for (size_t i = 0; i < cloud.size(); i++) {
			statisticalOutlierRemoval(cloud[i], cloud[i], OutlierK, OutlierTh);
		}
	}

	// DownSampling
	for (size_t i = 0; i < cloud.size(); i++) {
		uniformSampling(cloud[i], D, cloud[i]);
	}
	
	for (size_t i = 0; i < cloud.size(); i++) {
		bilateralFiltering(cloud[i], cloud[i]);
	}


	
	

	

	//座標展開
	for (size_t i = 0; i < cloud.size(); i++) {
		modifyCloudCoordinate(cloud[i]);
	}

	
	// Display 4D Data
	visualization::PCLVisualizer *viewer(new visualization::PCLVisualizer("4D Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	int current = 0, next = 1;
	bool show_points_(true);

	while (!viewer->wasStopped()) {
		printf("current : %d\n", current);

		viewer->spinOnce(TimeDelay);
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		if (show_points_) {
			viewer->addPointCloud(cloud[current], "cloud");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
		}

		current = next;
		if (next != (cloud.size() - 1)) next++;
		else next = 0;
	}

	return 0;
}