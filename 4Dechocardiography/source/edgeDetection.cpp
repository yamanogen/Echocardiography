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
	for (int i = 0; i < 1; i++) {
		constructPointXYZRGB(cloud_elem);
		cloudMake(cloud_elem, pixeldata, i + 1);
		cloud.push_back(cloud_elem);
	}

	/*vector<PointCloud<PointXYZRGB>::Ptr> cloud;
	PointCloud<PointXYZRGB>::Ptr cloud_elem;
	for (int i = 0; i < 1; i++) {
		constructPointXYZRGB(cloud_elem);
		if (io::loadPCDFile("milk_cartoon_all_small_clorox.pcd", *cloud_elem)==-1) {
			PCL_EIGEN_SIZE_MIN_PREFER_DYNAMIC("Couldn't read file\n");
			return -1;
		}
		cloud.push_back(cloud_elem);
	}*/

	//// DownSampling
	//for (size_t i = 0; i < cloud.size(); i++) {
	//	uniformSampling(cloud[i], D, cloud[i]);
	//}

	////外れ値処理
	//for (int n = 0; n < 3; n++) {
	//	for (size_t i = 0; i < cloud.size(); i++) {
	//		statisticalOutlierRemoval(cloud[i], cloud[i], OutlierK, OutlierTh);
	//	}
	//}

	//for (size_t i = 0; i < cloud.size(); i++) {
	//	convertHistgram(cloud[i], cloud[i]);
	//}

	// Compute the normals 
	vector<PointCloud<Normal>::Ptr> cloud_normals;
	PointCloud<Normal>::Ptr cloud_normals_elem;
	for (size_t i = 0; i < cloud.size(); i++) {
		constructNormal(cloud_normals_elem);
		normalsEstimation(cloud[i], cloud_normals_elem);
		cloud_normals.push_back(cloud_normals_elem);
	}

	//edge detection
	//vector<vector<PointIndices>> label_indices_v;
	vector<PointIndices> label_indices;
	PointCloud<PointXYZRGB>::Ptr occluding_edges(new PointCloud<PointXYZRGB>),
		occluded_edges(new PointCloud<PointXYZRGB>),
		nan_boundary_edges(new PointCloud<PointXYZRGB>),
		high_curvature_edges(new PointCloud<PointXYZRGB>),
		rgb_edges(new PointCloud<PointXYZRGB>);
	for (size_t i = 0; i < cloud.size(); i++) {
		//label_indices.clear();
		edgeDetection(cloud[i], cloud_normals[i], label_indices);
		//label_indices_v.push_back(label_indices);
		cerr << "cloud.size : " << cloud[i]->size() << "  label_indices[0] : " << label_indices[0].indices.size() << endl;
		cerr << "                   " << "  label_indices[1] : " << label_indices[1].indices.size() << endl;
		cerr << "                   " << "  label_indices[2] : " << label_indices[2].indices.size() << endl;
		cerr << "                   " << "  label_indices[3] : " << label_indices[3].indices.size() << endl;
		cerr << "                   " << "  label_indices[4] : " << label_indices[4].indices.size() << endl;
		copyPointCloud(*cloud[i], label_indices[0].indices, *nan_boundary_edges);
		copyPointCloud(*cloud[i], label_indices[1].indices, *occluding_edges);
		copyPointCloud(*cloud[i], label_indices[2].indices, *occluded_edges);
		copyPointCloud(*cloud[i], label_indices[3].indices, *high_curvature_edges);
		copyPointCloud(*cloud[i], label_indices[4].indices, *rgb_edges);
	}


	//// create cloud of kepoints
	//bool keypoints_from_curvature_(true);
	//vector<PointCloud<PointXYZRGB>::Ptr> cloud_keypoints;
	//PointCloud<PointXYZRGB>::Ptr cloud_keypoints_elem;
	//if (keypoints_from_curvature_) {
	//	vector<vector<float>> curvature_v;
	//	curvature_v.resize(cloud.size());

	//	for (size_t i = 0; i < cloud.size(); i++) {
	//		constructPointXYZRGB(cloud_keypoints_elem);
	//		createKeypoints(cloud[i], curvature_v[i], cloud_keypoints_elem);
	//		cloud_keypoints.push_back(cloud_keypoints_elem);
	//	}
	//	for (size_t i = 0; i < cloud.size(); i++) {
	//		distributionMapCurvatures(curvature_v[i]);
	//	}
	//}
	//else {
	//	for (size_t i = 0; i < cloud.size(); i++) {
	//		constructPointXYZRGB(cloud_keypoints_elem);
	//		uniformSampling(cloud[i], D, cloud_keypoints_elem);
	//		cloud_keypoints.push_back(cloud_keypoints_elem);
	//	}
	//}

	////座標展開
	//for (size_t i = 0; i < cloud.size(); i++) {
	//	modifyCloudCoordinate(cloud[i]);
	//	//modifyCloudCoordinate(cloud_keypoints[i]);
	//}

	//for (size_t i = 0; i < cloud.size(); i++) {
	//	modifyCloudCoordinate(nan_boundary_edges);
	//	modifyCloudCoordinate(occluding_edges);
	//	modifyCloudCoordinate(occluded_edges);
	//	modifyCloudCoordinate(high_curvature_edges);
	//	modifyCloudCoordinate(rgb_edges);
	//}

	// Display 4D Data
	visualization::PCLVisualizer *viewer(new visualization::PCLVisualizer("4D Viewer"));

	
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	while (!viewer->wasStopped()) {

		viewer->spinOnce(TimeDelay);
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		const int point_size = 2;
		viewer->addPointCloud<PointXYZRGB> (nan_boundary_edges, "nan boundary edges"); 
		viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "nan boundary edges"); 
		viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.0f, 1.0f, "nan boundary edges"); 
		
		viewer->addPointCloud<PointXYZRGB>(occluding_edges, "occluding edges");
		viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "occluding edges");
		viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "occluding edges");
		
		viewer->addPointCloud<PointXYZRGB> (occluded_edges, "occluded edges"); 
		viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "occluded edges"); 
		viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "occluded edges"); 
		 
		viewer->addPointCloud<PointXYZRGB> (high_curvature_edges, "high curvature edges"); 
		viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "high curvature edges"); 
		viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 0.0f, "high curvature edges"); 
		
		viewer->addPointCloud<PointXYZRGB>(rgb_edges, "rgb edges");
		viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "rgb edges");
		viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 1.0f, "rgb edges");

	}

	return 0;
}