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

	//point cloud declare and make
	vector<PointCloud<PointXYZRGB>::Ptr> cloud;
	PointCloud<PointXYZRGB>::Ptr cloud_elem;
	for (int i = 0; i < 10; i++) {
		constructPointXYZRGB(cloud_elem);
		cloudMake(cloud_elem, pixeldata, i + 1);
		cloud.push_back(cloud_elem);
	}
	cout << "cloud.size() : " << cloud.size() << endl;

	//範囲指定除去

	//外れ値処理
	for (int n = 0; n < 2; n++) {
		for (size_t i = 0; i < cloud.size(); i++) {
			statisticalOutlierRemoval(cloud[i], cloud[i], 50, OutlierTh);
		}
	}

	//ダウンサンプリング
	for (size_t i = 0; i < cloud.size(); i++) {
		uniformSampling(cloud[i], D, cloud[i]);
	}

	// Compute the normals 
	vector<PointCloud<Normal>::Ptr> cloud_normals;
	PointCloud<Normal>::Ptr cloud_normals_elem;
	for (size_t i = 0; i < cloud.size(); i++) {
		constructNormal(cloud_normals_elem);
		normalsEstimation(cloud[i], cloud_normals_elem);
		cloud_normals.push_back(cloud_normals_elem);
	}

	// create cloud of kepoints
	bool keypoints_from_curvature_(true);
	vector<PointCloud<PointXYZRGB>::Ptr> cloud_keypoints;
	PointCloud<PointXYZRGB>::Ptr cloud_keypoints_elem;
	if (keypoints_from_curvature_) {
		vector<vector<float>> curvature_v;
		curvature_v.resize(cloud.size());

		for (size_t i = 0; i < cloud.size(); i++) {
			constructPointXYZRGB(cloud_keypoints_elem);
			createKeypoints(cloud[i], curvature_v[i], cloud_keypoints_elem);
			cloud_keypoints.push_back(cloud_keypoints_elem);
		}
		/*for (size_t i = 0; i < cloud.size(); i++) {
		distributionMapCurvatures(curvature_v[i]);
		}*/
	}
	else {
		for (size_t i = 0; i < cloud.size(); i++) {
			constructPointXYZRGB(cloud_keypoints_elem);
			uniformSampling(cloud[i], D, cloud_keypoints_elem);
			cloud_keypoints.push_back(cloud_keypoints_elem);
		}
	}

	//compute SHOT
	vector<PointCloud<SHOT352>::Ptr> cloud_descriptors;
	PointCloud<SHOT352>::Ptr cloud_descriptors_elem;
	for (size_t i = 0; i < cloud.size(); i++) {
		constructSHOT352(cloud_descriptors_elem);
		calculateSHOTOMP(cloud[i], cloud_normals[i], cloud_keypoints[i], cloud_descriptors_elem);
		cloud_descriptors.push_back(cloud_descriptors_elem);
	}

	//correspondences
	vector<vector<Correspondence>> cloud_corrs;
	cloud_corrs.resize(cloud.size());
	for (size_t i = 0; i < cloud.size(); i++) {
		if (i != (cloud.size() - 1)) matchDescriptors(cloud_keypoints[i], cloud_keypoints[i + 1], cloud_descriptors[i], cloud_descriptors[i + 1], cloud_corrs[i]);
		else matchDescriptors(cloud_keypoints[i], cloud_keypoints[0], cloud_descriptors[i], cloud_descriptors[0], cloud_corrs[i]);
	}

	cout << "Correspondences found: " << cloud_corrs[0].size() << endl;

	//座標展開
	for (size_t i = 0; i < cloud.size(); i++) {
		modifyCloudCoordinate(cloud[i]);
		modifyCloudCoordinate(cloud_keypoints[i]);
	}

	//4Dデータ表示
	visualization::PCLVisualizer *viewer(new visualization::PCLVisualizer("3D Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	int current = 0, next = 1;
	bool show_points_(false);
	bool show_keypoints_(true);
	bool show_correspondences_(true);

	while (!viewer->wasStopped()) {
		printf("current : %d\n", current);

		viewer->spinOnce(TimeDelay);
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		if (show_points_) {
			viewer->addPointCloud(cloud[current], "cloud");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
		}
		if (show_keypoints_) {
			viewer->addPointCloud(cloud_keypoints[current], "cloud_keypoints");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_keypoints");
		}
		if (show_correspondences_) {
			for (size_t i = 0; i < cloud_corrs[current].size(); i += 10) {
				stringstream ss_line;
				ss_line << "correspondence_line" << i;

				PointXYZRGB& cloudA_point = cloud_keypoints[current]->at(cloud_corrs[current][i].index_match);
				PointXYZRGB& cloudB_point = cloud_keypoints[next]->at(cloud_corrs[current][i].index_query);

				//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
				viewer->addLine<PointXYZRGB, PointXYZRGB>(cloudA_point, cloudB_point, 0, 255, 0, ss_line.str());
				//viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH, 1, ss_line.str());
			}
		}

		current = next;
		if (next != (cloud.size() - 1)) next++;
		else next = 0;
	}

	return 0;
}