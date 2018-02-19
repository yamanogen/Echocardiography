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


	pcl::PointCloud<pcl::PointNormal>::Ptr vector1(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr vector2(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr vector3(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr vector4(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr vector5(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr vector6(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr vector7(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr vector8(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr vector9(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr vector10(new pcl::PointCloud<pcl::PointNormal>);


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
	cerr << "cloud1: " << cloud1->size() << endl;

	vectorMake(vector1, 1);
	vectorMake(vector2, 2);
	vectorMake(vector3, 3);
	vectorMake(vector4, 4);
	vectorMake(vector5, 5);
	vectorMake(vector6, 6);
	vectorMake(vector7, 7);
	vectorMake(vector8, 8);
	vectorMake(vector9, 9);
	vectorMake(vector10, 10);

	std::cerr << "vector1: " << vector1->size() << std::endl;
	//printVector(vector1);

	///////////////cloudデータ処理///////////////////



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
	OutlierException(cloud10, cloud10, 50, OutlierTh);
	cerr << "cloud1: " << cloud1->size() << endl;

	//ダウンサンプリング
	DownSampling(cloud1, cloud1, D / 2, D / 2, D / 2);
	DownSampling(cloud2, cloud2, D / 2, D / 2, D / 2);
	DownSampling(cloud3, cloud3, D / 2, D / 2, D / 2);
	DownSampling(cloud4, cloud4, D / 2, D / 2, D / 2);
	DownSampling(cloud5, cloud5, D / 2, D / 2, D / 2);
	DownSampling(cloud6, cloud6, D / 2, D / 2, D / 2);
	DownSampling(cloud7, cloud7, D / 2, D / 2, D / 2);
	DownSampling(cloud8, cloud8, D / 2, D / 2, D / 2);
	DownSampling(cloud9, cloud9, D / 2, D / 2, D / 2);
	DownSampling(cloud10, cloud10, D / 2, D / 2, D / 2);

	//外れ値処理

	//座標修正


	//CSV出力

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
	modifyVectorCoordinate(vector1);
	modifyVectorCoordinate(vector2);
	modifyVectorCoordinate(vector3);
	modifyVectorCoordinate(vector4);
	modifyVectorCoordinate(vector5);
	modifyVectorCoordinate(vector6);
	modifyVectorCoordinate(vector7);
	modifyVectorCoordinate(vector8);
	modifyVectorCoordinate(vector9);
	modifyVectorCoordinate(vector10);

	//4Dデータ表示
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	/*
	viewer->setBackgroundColor(0, 0, 0);
	addCloud(cloud1,viewer);
	//addVectorLine(vector1, viewer);
	//addCloud(cloud1, viewer);
	//addVectorArrow(vector1, viewer);

	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	*/
	
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	int cloudNumber = 1;
	/*
	while (!viewer->wasStopped()) {
		
		viewer->spinOnce(100);
		switch (cloudNumber) {
			case 1: addCloud(cloud1, viewer); addVectorLine(vector1, viewer); break;
			case 2: addCloud(cloud2, viewer); addVectorLine(vector2, viewer); break;
			case 3: addCloud(cloud3, viewer); addVectorLine(vector3, viewer); break;
			case 4: addCloud(cloud4, viewer); addVectorLine(vector4, viewer); break;
			case 5: addCloud(cloud5, viewer); addVectorLine(vector5, viewer); break;
			case 6: addCloud(cloud6, viewer); addVectorLine(vector6, viewer); break;
			case 7: addCloud(cloud7, viewer); addVectorLine(vector7, viewer); break;
			case 8: addCloud(cloud8, viewer); addVectorLine(vector8, viewer); break;
			case 9: addCloud(cloud9, viewer); addVectorLine(vector9, viewer); break;
			case 10: addCloud(cloud10, viewer); addVectorLine(vector10, viewer); break;
			default : break;
		}
		Sleep(TimeDelay);
		viewer->removePointCloud("cloud", 0);


		switch (cloudNumber) {
		case 1: removeVectorLine(vector1, viewer); cloudNumber++; break;
		case 2: removeVectorLine(vector2, viewer); cloudNumber++; break;
		case 3: removeVectorLine(vector3, viewer); cloudNumber++; break;
		case 4: removeVectorLine(vector4, viewer); cloudNumber++; break;
		case 5: removeVectorLine(vector5, viewer); cloudNumber++; break;
		case 6: removeVectorLine(vector6, viewer); cloudNumber++; break;
		case 7: removeVectorLine(vector7, viewer); cloudNumber++; break;
		case 8: removeVectorLine(vector8, viewer); cloudNumber++; break;
		case 9: removeVectorLine(vector9, viewer); cloudNumber++; break;
		case 10: removeVectorLine(vector10, viewer); cloudNumber = 1; break;
		default: break;
		}

	}
	*/

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_view(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointNormal>::Ptr vector_view(new pcl::PointCloud<pcl::PointNormal>);

	while (!viewer->wasStopped()) {

		viewer->spinOnce(TimeDelay);
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		switch (cloudNumber) {
		case 1: cloud_view = cloud1; vector_view = vector1; break;
		case 2: cloud_view = cloud2; vector_view = vector2; break;
		case 3: cloud_view = cloud3; vector_view = vector3; break;
		case 4: cloud_view = cloud4; vector_view = vector4; break;
		case 5: cloud_view = cloud5; vector_view = vector5; break;
		case 6: cloud_view = cloud6; vector_view = vector6; break;
		case 7: cloud_view = cloud7; vector_view = vector7; break;
		case 8: cloud_view = cloud8; vector_view = vector8; break;
		case 9: cloud_view = cloud9; vector_view = vector9; break;
		case 10: cloud_view = cloud10; vector_view = vector10; cloudNumber = 0; break;
		default: break;
		}
		
		printf("%d\n", cloudNumber);
		cloudNumber++;
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud_view, "cloud");
		addVectorLine(vector_view, viewer);
		//Sleep(TimeDelay * 2);
		
	}

	return 0;
}