#include "stdafx.h"
#include "MyHeader.h"

VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

//frame 10 ok
char DCMfilenameTest1[256] = "C:\\Users\\guest1\\Desktop\\心臓データ\\Echocardiography\\qlab\\1\\A_11Jul2017_1.2.840.113663.1500.1.327662638.3.1.20170711.180346.3752_.dcm";
//frame 12
char DCMfilenameGoal1[256] = "C:\\Users\\guest1\\Desktop\\心臓データ\\sinecho\\goal\\3dCartesian\\1\\A_15Sep2017_1.2.840.113663.1500.1.327662638.3.1.20170915.221218.234_.dcm";
//frame 12
char DCMfilenameGoal2[256] = "C:\\Users\\guest1\\Desktop\\心臓データ\\sinecho\\goal\\3dCartesian\\2\\A_15Sep2017_1.2.840.113663.1500.1.327662638.3.2.20170915.221807.859_.dcm";
//frame 12
char DCMfilenameGoal3[256] = "C:\\Users\\guest1\\Desktop\\心臓データ\\sinecho\\goal\\3dCartesian\\3\\A_15Sep2017_1.2.840.113663.1500.1.327662638.3.3.20170915.222310.46_.dcm";

int main(void)
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

	//DICOMからピクセルデータ取得
	DcmFileFormat dcmfile;
	DcmDataset *dataset;
	DcmElement *element;
	Uint8 *pixeldata;
	if (dcmfile.loadFile(DCMfilenameTest1).bad()) {
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
	cloudMake2(cloud1, pixeldata, 1);
	cloudMake2(cloud2, pixeldata, 2);
	cloudMake2(cloud3, pixeldata, 3);
	cloudMake2(cloud4, pixeldata, 4);
	cloudMake2(cloud5, pixeldata, 5);
	cloudMake2(cloud6, pixeldata, 6);
	cloudMake2(cloud7, pixeldata, 7);
	cloudMake2(cloud8, pixeldata, 8);
	cloudMake2(cloud9, pixeldata, 9);
	cloudMake2(cloud10, pixeldata, 10);

	std::cerr << "cloud1: " << cloud1->size() << std::endl;

	//cloudデータ処理	

	//範囲指定除去

	//座標展開
	modifyCloudCoordinate(cloud1);
	modifyCloudCoordinate(cloud2);
	modifyCloudCoordinate(cloud3);
	modifyCloudCoordinate(cloud4);
	modifyCloudCoordinate(cloud5);
	modifyCloudCoordinate(cloud6);
	modifyCloudCoordinate(cloud7);
	modifyCloudCoordinate(cloud8);
	modifyCloudCoordinate(cloud9);
	//modifyCloudCoordinate(cloud10);


	//外れ値処理

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

	OutlierException(cloud1, cloud1, 50, OutlierTh);
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
	
	DownSampling(cloud1, cloud1, D*PixelSpacingX, D*PixelSpacingY, D*PixelSpacingZ);
	/*
	DownSampling(cloud2, cloud2, D, D, D);
	DownSampling(cloud3, cloud3, D, D, D);
	DownSampling(cloud4, cloud4, D, D, D);
	DownSampling(cloud5, cloud5, D, D, D);
	DownSampling(cloud6, cloud6, D, D, D);
	DownSampling(cloud7, cloud7, D, D, D);
	DownSampling(cloud8, cloud8, D, D, D);
	DownSampling(cloud9, cloud9, D, D, D);
	DownSampling(cloud10, cloud10, D, D, D);
	*/

	//sift計算
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
	cloud_normals = Surface_normals(cloud1);

	//　XYZの情報をcloudからSurface_normals(cloud)にXYZとして加える
	for (size_t i = 0; i < cloud_normals->points.size(); ++i)
	{
		cloud_normals->points[i].x = cloud1->points[i].x;
		cloud_normals->points[i].y = cloud1->points[i].y;
		cloud_normals->points[i].z = cloud1->points[i].z;
	}

	// 視覚化のためSIFT計算の結果をcloud_tempにコピー
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	copyPointCloud(Extract_SIFT(cloud1, cloud_normals), *cloud_temp);
	std::cout << "SIFT points in the cloud_temp are " << cloud_temp->points.size() << std::endl;
	//外れ値処理

	//セグメンテーション

	//投影

	//座標修正

	
	//4Dデータ表示
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> keypoints_color_handler(cloud_temp, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler(cloud1, 255, 255, 255);
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	//viewer.addPointCloud(cloud1, "cloud");
	viewer.addPointCloud(cloud_temp, keypoints_color_handler, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "keypoints");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	/*
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_view(new pcl::PointCloud<pcl::PointXYZRGB>);
	int cloudNumber = 1;

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	while (!viewer->wasStopped()) {

	viewer->spinOnce(TimeDelay);
	viewer->removeAllPointClouds();
	viewer->removeAllShapes();

	switch (cloudNumber) {
	case 1: cloud_view = cloud1; break;
	case 2: cloud_view = cloud2; break;
	case 3: cloud_view = cloud3; break;
	case 4: cloud_view = cloud4; break;
	case 5: cloud_view = cloud5; break;
	case 6: cloud_view = cloud6; break;
	case 7: cloud_view = cloud7; break;
	case 8: cloud_view = cloud8; break;
	case 9: cloud_view = cloud9; break;
	case 10: cloud_view = cloud10; break;
	default: break;
	}

	printf("%d\n", cloudNumber);
	if (cloudNumber == 9) cloudNumber = 1;
	else cloudNumber++;
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_view, "cloud");

	//Sleep(TimeDelay * 2);

	}
	*/
	
	return 0;
}