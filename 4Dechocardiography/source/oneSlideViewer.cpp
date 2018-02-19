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
	if (dcmfile.loadFile(DCMfilenameTest13_1).bad()) {
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
	for (int i = 0; i < 7; i++) {//more than 1
		constructPointXYZRGB(cloud_elem);
		cloudMake(cloud_elem, pixeldata, i + 1);
		cloud.push_back(cloud_elem);
	}

	//座標展開
	for (size_t f = 0; f < cloud.size(); f++) {
		modifyCloudCoordinate(cloud[f]);
	}

	// Display 4D Data
	visualization::PCLVisualizer *viewer(new visualization::PCLVisualizer("4D Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	int current = 0, next = 1;
	bool show_points_(true);
	bool show_splitedCloud_Line_(false);
	bool show_info_(true);

	int count = 0;

	PointXYZRGB p0;
	p0.x = 0; p0.y = 0; p0.z = 0;

	int theta = 0;
	int Z = 0, Y = 0, X = 0;

	while (!viewer->wasStopped()) {
		printf("current : %d\n", current);
		viewer->spinOnce(TimeDelay);
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		if (show_points_) {
			viewer->addPointCloud(cloud[current], "cloud");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "cloud");
		}

		char text[16], text2[64];
		sprintf_s(text, "Frame:%d", current);
		viewer->addText(text, 100, 0, 30, 1.0, 1.0, 1.0, "Frame");

		//sprintf_s(text2, "Mitral Valve  X:%d Y:%d Z:%d", X, Y, Z);
		sprintf_s(text2, "Aortic Valve  X:%d Y:%d Z:%d", X, Y, Z);
		viewer->addText(text2, 300, 0, 30, 1.0, 1.0, 1.0, "cartesian");

		if (show_info_) {
			if (current == 0 && count == 0) {
				//cout << "Input number of theta" << endl;

				//cout << "-45~45 : theta = "; cin >> theta;

				cout << "Input number of X,Y,Z" << endl;

				cout << "0~" << cols / div3 << " : X = "; cin >> X;
				cout << "0~" << rows / div3 << " : Y = "; cin >> Y;
				cout << "0~" << levs / div3 << " : Z = "; cin >> Z;

				count = 2;
			}
			//oneSlideView(viewer, cloud_smallgrid_div3[current], X, Y, Z);
			PointCloud<PointXYZRGB>::Ptr cloud_filtered(new PointCloud<PointXYZRGB>);
			PointCloud<PointXYZRGB>::Ptr cloud_filtered2(new PointCloud<PointXYZRGB>);
			createOneSlice(cloud[current], cloud_filtered, X, Y, Z, theta);
			cout << "cloud_filtered: " << cloud_filtered->size() << endl;
			transformPointCloud(*cloud_filtered, *cloud_filtered2, Eigen::Vector3f(0, 200 * PixelSpacingY, 0), Eigen::Quaternionf(1, 0, 0, 0));
			viewer->addPointCloud(cloud_filtered, "cloud_filtered");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "cloud_filtered");
			viewer->addPointCloud(cloud_filtered2, "cloud_filtered2");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "cloud_filtered2");

			PointCloud<PointXYZ>::Ptr P(new PointCloud<PointXYZ>);
			PointXYZ p;
			p.x = X*div3*PixelSpacingX;
			p.y = Y*div3*PixelSpacingY + 200 * PixelSpacingY;
			p.z = Z*div3*PixelSpacingZ;
			P->push_back(p);
			viewer->addPointCloud(P, "P");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 25, "P");
		}

		current = next;
		if (next != (cloud.size() - 1)) {
			next++;
		}
		else {
			next = 0;
			count--;
		}
	}

	return 0;
}