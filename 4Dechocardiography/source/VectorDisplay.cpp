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
	std::cerr << "cloud1: " << cloud1->size() << std::endl;

	vectorMake(vector1, 1);
	/*
	vectorMake(vector2, 2);
	vectorMake(vector3, 3);
	vectorMake(vector4, 4);
	vectorMake(vector5, 5);
	vectorMake(vector6, 6);
	vectorMake(vector7, 7);
	vectorMake(vector8, 8);
	vectorMake(vector9, 9);
	vectorMake(vector10, 10);
	*/

	std::cerr << "vector1: " << vector1->size() << std::endl;
	printVector(vector1);

	///////////////cloudデータ処理///////////////////



	//外れ値処理

	//ダウンサンプリング

	//外れ値処理

	//座標修正

	//CSV出力

	//座標展開

	//4Dデータ表示
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(255, 255, 255);
	//viewer->addPointCloud<pcl::PointXYZRGB>(cloud1, "cloud1");
	viewer->addPointCloudNormals<pcl::PointNormal>(vector1, 100, 20, "vector1");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


	return 0;
}