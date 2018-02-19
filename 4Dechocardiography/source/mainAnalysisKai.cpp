#include "stdafx.h"
#include "MyHeader.h"

VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

//frame 10 ok
char DCMfilenameTest1[256] = "C:\\Users\\guest1\\Desktop\\心臓データ\\Echocardiography\\qlab\\1\\A_11Jul2017_1.2.840.113663.1500.1.327662638.3.1.20170711.180346.3752_.dcm";
//frame 21
char DCMfilenameTest2[256] = "C:\\Users\\guest1\\Desktop\\心臓データ\\echocardio\\x7-250deg60degCenter\\A_11Sep2017_1.2.840.113663.1500.1.327662638.3.1.20170911.190600.671_.dcm";
//frame 9
char DCMfilenameTest3[256] = "C:\\Users\\guest1\\Desktop\\心臓データ\\echocardio\\x7-2_72deg100deg_Center\\A_11Sep2017_1.2.840.113663.1500.1.327662638.3.1.20170911.191345.843_.dcm";
//frame 13
char DCMfilenameTest4[256] = "C:\\Users\\guest1\\Desktop\\心臓データ\\echocardio\\x3-1_48deg50deg_Center\\A_11Sep2017_1.2.840.113663.1500.1.327662638.3.1.20170911.193525.859_2.dcm";
//frame 20 ok
char DCMfilenameOther[256] = "C:\\Users\\guest1\\Desktop\\心臓データ\\sinecho\\other\\A_15Sep2017_1.2.840.113663.1500.1.327662638.3.1.20170915.222916.953_.dcm";
//frame 12
char DCMfilenameGoal1[256] = "C:\\Users\\guest1\\Desktop\\心臓データ\\sinecho\\goal\\3dCartesian\\1\\A_15Sep2017_1.2.840.113663.1500.1.327662638.3.1.20170915.221218.234_.dcm";
//frame 12
char DCMfilenameGoal2[256] = "C:\\Users\\guest1\\Desktop\\心臓データ\\sinecho\\goal\\3dCartesian\\2\\A_15Sep2017_1.2.840.113663.1500.1.327662638.3.2.20170915.221807.859_.dcm";
//frame 12
char DCMfilenameGoal3[256] = "C:\\Users\\guest1\\Desktop\\心臓データ\\sinecho\\goal\\3dCartesian\\3\\A_15Sep2017_1.2.840.113663.1500.1.327662638.3.3.20170915.222310.46_.dcm";
//frame 12
char DCMfilenameFail1[256] = "C:\\Users\\guest1\\Desktop\\心臓データ\\sinecho\\fail\\3dCartesian\\1\\A_15Sep2017_1.2.840.113663.1500.1.327662638.3.1.20170915.214725.421_.dcm";
//frame 12
char DCMfilenameFail2[256] = "C:\\Users\\guest1\\Desktop\\心臓データ\\sinecho\\fail\\3dCartesian\\2\\A_15Sep2017_1.2.840.113663.1500.1.327662638.3.2.20170915.214755.890_.dcm";
//frame 12
char DCMfilenameFail3[256] = "C:\\Users\\guest1\\Desktop\\心臓データ\\sinecho\\fail\\3dCartesian\\3\\A_15Sep2017_1.2.840.113663.1500.1.327662638.3.3.20170915.214831.765_.dcm";
//frame 12
char DCMfilenameFail4[256] = "C:\\Users\\guest1\\Desktop\\心臓データ\\sinecho\\fail\\3dCartesian\\4\\A_15Sep2017_1.2.840.113663.1500.1.327662638.3.4.20170915.214845.125_.dcm";


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
	if (dcmfile.loadFile(DCMfilenameGoal2).bad()) {
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
	/*
	cloudMake2(cloud11, pixeldata, 11);
	cloudMake2(cloud12, pixeldata, 12);
	cloudMake2(cloud13, pixeldata, 13);
	cloudMake2(cloud14, pixeldata, 14);
	cloudMake2(cloud15, pixeldata, 15);
	*/

	//cloudMakeLeft(cloudL, pixeldata, 1);
	//cloudMakeRight(cloudR, pixeldata, 1);

	std::cerr << "cloud1: " << cloud1->size() << std::endl;

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
	
	OutlierException(cloud1, cloud1, 50, OutlierTh);
	OutlierException(cloud2, cloud2, 50, OutlierTh);
	OutlierException(cloud3, cloud3, 50, OutlierTh);
	OutlierException(cloud4, cloud4, 50, OutlierTh);
	OutlierException(cloud5, cloud5, 50, OutlierTh);
	OutlierException(cloud6, cloud6, 50, OutlierTh);
	OutlierException(cloud7, cloud7, 50, OutlierTh);
	OutlierException(cloud8, cloud8, 50, OutlierTh);
	OutlierException(cloud9, cloud9, 50, OutlierTh);
	//OutlierException(cloud10, cloud10, 50, OutlierTh);
	
	
	

	//ダウンサンプリング
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	/*
	DownSampling(cloud1, cloud1, D, D, D);
	DownSampling(cloud2, cloud2, D, D, D);
	DownSampling(cloud3, cloud3, D, D, D);
	DownSampling(cloud4, cloud4, D, D, D);
	DownSampling(cloud5, cloud5, D, D, D);
	DownSampling(cloud6, cloud6, D, D, D);
	DownSampling(cloud7, cloud7, D, D, D);
	DownSampling(cloud8, cloud8, D, D, D);
	DownSampling(cloud9, cloud9, D, D, D);
	//DownSampling(cloud10, cloud10, D, D, D);
	*/

	//外れ値処理
	
	OutlierException(cloud1, cloud1, 50, OutlierTh);
	OutlierException(cloud2, cloud2, 50, OutlierTh);
	OutlierException(cloud3, cloud3, 50, OutlierTh);
	OutlierException(cloud4, cloud4, 50, OutlierTh);
	OutlierException(cloud5, cloud5, 50, OutlierTh);
	OutlierException(cloud6, cloud6, 50, OutlierTh);
	OutlierException(cloud7, cloud7, 50, OutlierTh);
	OutlierException(cloud8, cloud8, 50, OutlierTh);
	OutlierException(cloud9, cloud9, 50, OutlierTh);
	//OutlierException(cloud10, cloud10, 50, OutlierTh);
	

	//セグメンテーション
	//Segmentation(cloud1);

	//投影
	//projectPlane(cloudL);
	//projectPlane(cloudR);

	//座標修正
	/*
	compressCoordinate(cloud1);
	compressCoordinate(cloud2);
	compressCoordinate(cloud3);
	compressCoordinate(cloud4);
	compressCoordinate(cloud5);
	compressCoordinate(cloud6);
	compressCoordinate(cloud7);
	compressCoordinate(cloud8);
	compressCoordinate(cloud9);
	//compressCoordinate(cloud10);
	*/

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

	//4Dデータ表示

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

	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	while (!viewer.wasStopped()) {

		viewer.showCloud(cloud1);
		printf("1\n");
		Sleep(TimeDelay);

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
		//viewer.showCloud(cloud10);
		//printf("10\n");
		//Sleep(TimeDelay);
	}
	return 0;
}