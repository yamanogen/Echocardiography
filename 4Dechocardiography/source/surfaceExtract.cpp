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
	PointCloud<PointXYZRGB>::Ptr cloud1(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud2(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud3(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud4(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud5(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud6(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud7(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud8(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud9(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud10(new PointCloud<PointXYZRGB>);

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
	//cloudMake2(cloud2, pixeldata, 2);

	/*
	cloudMake2(cloud3, pixeldata, 3);
	cloudMake2(cloud4, pixeldata, 4);
	cloudMake2(cloud5, pixeldata, 5);
	cloudMake2(cloud6, pixeldata, 6);
	cloudMake2(cloud7, pixeldata, 7);
	cloudMake2(cloud8, pixeldata, 8);
	cloudMake2(cloud9, pixeldata, 9);
	cloudMake2(cloud10, pixeldata, 10);
	*/

	//範囲指定除去

	//外れ値処理

	//PointCloud<PointXYZRGB>::Ptr cloud1_filtered(new PointCloud<PointXYZRGB>);

	statisticalOutlierRemoval(cloud1, cloud1, 50, OutlierTh);
	//statisticalOutlierRemoval(cloud2, cloud2, 50, OutlierTh);
	/*
	statisticalOutlierRemoval(cloud3, cloud3, 50, OutlierTh);
	statisticalOutlierRemoval(cloud4, cloud4, 50, OutlierTh);
	statisticalOutlierRemoval(cloud5, cloud5, 50, OutlierTh);
	statisticalOutlierRemoval(cloud6, cloud6, 50, OutlierTh);
	statisticalOutlierRemoval(cloud7, cloud7, 50, OutlierTh);
	statisticalOutlierRemoval(cloud8, cloud8, 50, OutlierTh);
	statisticalOutlierRemoval(cloud9, cloud9, 50, OutlierTh);
	statisticalOutlierRemoval(cloud10, cloud10, 50, OutlierTh);
	*/

	statisticalOutlierRemoval(cloud1, cloud1, 50, OutlierTh);
	//statisticalOutlierRemoval(cloud2, cloud2, 50, OutlierTh);

	//ダウンサンプリング
	//PointCloud<PointXYZRGB>::Ptr cloud1_filtered(new PointCloud<PointXYZRGB>);

	DownSampling(cloud1, cloud1, D, D, D);
	//DownSampling(cloud2, cloud2, D, D, D);
	/*
	DownSampling(cloud3, cloud3, D, D, D);
	DownSampling(cloud4, cloud4, D, D, D);
	DownSampling(cloud5, cloud5, D, D, D);
	DownSampling(cloud6, cloud6, D, D, D);
	DownSampling(cloud7, cloud7, D, D, D);
	DownSampling(cloud8, cloud8, D, D, D);
	DownSampling(cloud9, cloud9, D, D, D);
	DownSampling(cloud10, cloud10, D, D, D);
	*/

	//外れ値処理
	statisticalOutlierRemoval(cloud1, cloud1, 50, OutlierTh);
	//statisticalOutlierRemoval(cloud2, cloud2, 50, OutlierTh);


	// Compute the normals 
	vector<float> curvature_v1, curvature_v2;
	PointCloud<Normal>::Ptr cloud1_normals(new PointCloud<Normal>);
	PointCloud<Normal>::Ptr cloud2_normals(new PointCloud<Normal>);
	normalsEstimation(cloud1, cloud1_normals);
	normalsEstimation(cloud2, cloud2_normals);

	// create cloud of kepoints
	PointCloud<PointXYZRGB>::Ptr cloud1_keypoints(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud2_keypoints(new PointCloud<PointXYZRGB>);
	createKeypoints(cloud1, curvature_v1, cloud1_keypoints);
	createKeypoints(cloud2, curvature_v2, cloud2_keypoints);

	distributionMapCurvatures(curvature_v1);
	//distributionMapCurvatures(curvature_v2);
	//setColorForEachCurvature(cloud1, curvature_v1);
	//setColorForEachCurvature(cloud2, curvature_v2);

	if (1) {
		visualization::PCLVisualizer *viewer(new visualization::PCLVisualizer("3D Viewer"));
		PointCloud<PointXYZRGB>::Ptr cloud_view(new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr cloud_keypoints_view(new PointCloud<PointXYZRGB>);
		int cloudNumber = 1;
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addCoordinateSystem(1.0);
		viewer->initCameraParameters();
		while (!viewer->wasStopped()) {
			viewer->spinOnce(TimeDelay);
			viewer->removeAllPointClouds();
			viewer->removeAllShapes();
			switch (cloudNumber) {
			case 1: cloud_view = cloud1; cloud_keypoints_view = cloud1_keypoints; break;
			case 2: cloud_view = cloud2; cloud_keypoints_view = cloud2_keypoints; break;
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
			//printf("%d\n", cloudNumber);
			if (cloudNumber == 1) cloudNumber = 1;
			else cloudNumber++;
			viewer->addPointCloud(cloud_keypoints_view, "cloud_keypoints");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_keypoints");
			//viewer->addPointCloud(cloud_view, "cloud");
			//viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
			//Sleep(TimeDelay * 2);
		}
		return 0;
	}


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
	modifyCloudCoordinate(cloud10);
	modifyCloudCoordinate(cloud1_keypoints);
	modifyCloudCoordinate(cloud2_keypoints);


	//4Dデータ表示
	/*
	//boost::shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer("3D Viewer"));
	visualization::PCLVisualizer *viewer(new visualization::PCLVisualizer("3D Viewer"));
	PointCloud<PointXYZRGB>::Ptr cloud_view(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud_keypoints_view(new PointCloud<PointXYZRGB>);
	int cloudNumber = 1;

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	while (!viewer->wasStopped()) {

	viewer->spinOnce(TimeDelay);
	viewer->removeAllPointClouds();
	viewer->removeAllShapes();

	switch (cloudNumber) {
	case 1: cloud_view = cloud1; cloud_keypoints_view = cloud1_keypoints; break;
	case 2: cloud_view = cloud2; cloud_keypoints_view = cloud2_keypoints; break;
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
	if (cloudNumber == 1) cloudNumber = 1;
	else cloudNumber++;

	viewer->addPointCloud(cloud_keypoints_view, "cloud_keypoints");
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_keypoints");
	viewer->addPointCloud(cloud_view, "cloud");
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

	//Sleep(TimeDelay * 2);

	}
	*/

	return 0;
}