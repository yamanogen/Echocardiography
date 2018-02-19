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
	if (dcmfile.loadFile(DCMfilenameGoal1).bad()) {
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

	//範囲指定除去

	// DownSampling
	for (size_t i = 0; i < cloud.size(); i++) {
		uniformSampling(cloud[i], D, cloud[i]);
	}

	//外れ値処理
	for (int n = 0; n < 2; n++) {
		for (size_t i = 0; i < cloud.size(); i++) {
			statisticalOutlierRemoval(cloud[i], cloud[i], OutlierK, OutlierTh);
		}
	}

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

	for (size_t i = 0; i < cloud.size(); i++) {
		bilateralFiltering(cloud[i], cloud[i]);
	}

	//convert Histgram
	for (size_t i = 0; i < cloud.size(); i++) {
		convertHistgram(cloud[i], cloud[i]);
	}

	// Compute the normals 
	/*vector<PointCloud<Normal>::Ptr> cloud_normals;
	PointCloud<Normal>::Ptr cloud_normals_elem;
	for (size_t i = 0; i < cloud.size(); i++) {
		constructNormal(cloud_normals_elem);
		normalsEstimation(cloud[i], cloud_normals_elem);
		cloud_normals.push_back(cloud_normals_elem);
	}*/

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
			uniformSampling(cloud[i], D * 2, cloud_keypoints_elem);
			cloud_keypoints.push_back(cloud_keypoints_elem);
		}
	}

	// DownSampling
	for (size_t i = 0; i < cloud.size(); i++) {
		uniformSampling(cloud_keypoints[i], DofKeypoints, cloud_keypoints[i]);
	}

	// Compute Normals
	vector<PointCloud<Normal>::Ptr> cloud_normals;
	PointCloud<Normal>::Ptr cloud_normals_elem;
	for (size_t i = 0; i < cloud.size(); i++) {
		constructNormal(cloud_normals_elem);
		cloud_normals.push_back(cloud_normals_elem);
	}
	for (size_t i = 0; i < cloud.size(); i++) {
		normalsEstimation(cloud[i], cloud_normals[i]);
	}

	// Compute SHOT
	vector<PointCloud<SHOT352>::Ptr> cloud_descriptorsSHOT;
	PointCloud<SHOT352>::Ptr cloud_descriptorsSHOT_elem;
	for (size_t i = 0; i < cloud.size(); i++) {
		constructSHOT352(cloud_descriptorsSHOT_elem);
		calculateSHOTOMP(cloud[i], cloud_normals[i], cloud_keypoints[i], cloud_descriptorsSHOT_elem);
		cloud_descriptorsSHOT.push_back(cloud_descriptorsSHOT_elem);
	}

	//correspondences
	vector<vector<Correspondence>> cloud_corrs;
	cloud_corrs.resize(cloud.size());
	vector<vector<Correspondence>> cloud_corrs_forward, cloud_corrs_reverse;
	cloud_corrs_forward.resize(cloud.size()); cloud_corrs_reverse.resize(cloud.size());
	
	for (size_t i = 0; i < cloud.size(); i++) {
		if (i != (cloud.size() - 1)) {
			matchDescriptors(cloud_keypoints[i], cloud_keypoints[i + 1], cloud_descriptorsSHOT[i], cloud_descriptorsSHOT[i + 1], cloud_corrs_forward[i]);
			matchDescriptors(cloud_keypoints[i + 1], cloud_keypoints[i], cloud_descriptorsSHOT[i + 1], cloud_descriptorsSHOT[i], cloud_corrs_reverse[i]);
		}
		else {
			matchDescriptors(cloud_keypoints[i], cloud_keypoints[0], cloud_descriptorsSHOT[i], cloud_descriptorsSHOT[0], cloud_corrs_forward[i]);
			matchDescriptors(cloud_keypoints[0], cloud_keypoints[i], cloud_descriptorsSHOT[0], cloud_descriptorsSHOT[i], cloud_corrs_reverse[i]);
		}
		cout << "Correspondences_forward[" << i << "] found: " << cloud_corrs_forward[i].size() << endl;
		cout << "Correspondences_reverse[" << i << "] found: " << cloud_corrs_reverse[i].size() << endl;
	}
	for (size_t i = 0; i < cloud.size(); i++) {
		reversibleDescriptorMatching(cloud_corrs_forward[i], cloud_corrs_reverse[i], cloud_corrs[i]);
		cout << "Correspondences[" << i << "] found: " << cloud_corrs[i].size() << endl;
	}

	//D or S
	vector<vector<int>> timing;
	timing.resize(cloud.size());
	timing[0].push_back(0); timing[0].push_back(0);
	timing[1].push_back(0); timing[1].push_back(1);
	timing[2].push_back(0); timing[2].push_back(2);
	timing[3].push_back(0); timing[3].push_back(3);
	timing[4].push_back(1); timing[4].push_back(0);
	timing[5].push_back(1); timing[5].push_back(1);
	timing[6].push_back(1); timing[6].push_back(2);
	timing[7].push_back(1); timing[7].push_back(3);

	//split cloud into cloud_smallgrid_div1 div1=4
	vector<vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>>> cloud_smallgrid_div1;
	PointCloud<PointXYZRGB>::Ptr cloud_smallgrid_div1_elem;
	cloud_smallgrid_div1.resize(cloud.size());

	vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> cloud_smallgrid_div1_vector;
	PointCloud<PointXYZRGBNormal>::Ptr cloud_smallgrid_div1_vector_elem;
	cloud_smallgrid_div1_vector.resize(cloud.size());

	for (size_t f = 0; f < cloud.size(); f++) {
		cloud_smallgrid_div1[f].resize(levs / div1);
		cloud_smallgrid_div1_vector[f].resize(levs / div1);
		for (int k = 0; k < levs / div1; k++) {
			cloud_smallgrid_div1[f][k].resize(rows / div1);
			cloud_smallgrid_div1_vector[f][k].resize(rows / div1);
			for (int j = 0; j < rows / div1; j++) {
				cloud_smallgrid_div1[f][k][j].resize(cols / div1);
				cloud_smallgrid_div1_vector[f][k][j].resize(cols / div1);
				for (int i = 0; i < cols / div1; i++) {
					constructPointXYZRGB(cloud_smallgrid_div1_elem);
					cloud_smallgrid_div1[f][k][j][i] = cloud_smallgrid_div1_elem;
					constructPointXYZRGBNormal(cloud_smallgrid_div1_vector_elem);
					cloud_smallgrid_div1_vector[f][k][j][i] = cloud_smallgrid_div1_vector_elem;
				}
			}
		}
		if (f != (cloud.size() - 1)) {
			splitCloudintoCloudSmallGrid(cloud[f], cloud_smallgrid_div1[f], cloud_keypoints[f], cloud_keypoints[f + 1], cloud_corrs[f], cloud_smallgrid_div1_vector[f]);
		}
		else {
			splitCloudintoCloudSmallGrid(cloud[f], cloud_smallgrid_div1[f], cloud_keypoints[f], cloud_keypoints[0], cloud_corrs[f], cloud_smallgrid_div1_vector[f]);
		}
	}

	//split cloud into cloud_smallgrid_div2 div2=8
	vector<vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>>> cloud_smallgrid_div2;
	PointCloud<PointXYZRGB>::Ptr cloud_smallgrid_div2_elem;
	cloud_smallgrid_div2.resize(cloud.size());

	vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> cloud_smallgrid_div2_vector;
	PointCloud<PointXYZRGBNormal>::Ptr cloud_smallgrid_div2_vector_elem;
	cloud_smallgrid_div2_vector.resize(cloud.size());

	for (size_t f = 0; f < cloud.size(); f++) {
		cloud_smallgrid_div2[f].resize(levs / div2);
		cloud_smallgrid_div2_vector[f].resize(levs / div2);
		for (int k = 0; k < levs / div2; k++) {
			cloud_smallgrid_div2[f][k].resize(rows / div2);
			cloud_smallgrid_div2_vector[f][k].resize(rows / div2);
			for (int j = 0; j < rows / div2; j++) {
				cloud_smallgrid_div2[f][k][j].resize(cols / div2);
				cloud_smallgrid_div2_vector[f][k][j].resize(cols / div2);
				for (int i = 0; i < cols / div2; i++) {
					constructPointXYZRGB(cloud_smallgrid_div2_elem);
					cloud_smallgrid_div2[f][k][j][i] = cloud_smallgrid_div2_elem;
					constructPointXYZRGBNormal(cloud_smallgrid_div2_vector_elem);
					cloud_smallgrid_div2_vector[f][k][j][i] = cloud_smallgrid_div2_vector_elem;
				}
			}
		}
		if (f != (cloud.size() - 1)) {
			splitCloudintoCloudSmallGrid2(cloud[f], cloud_smallgrid_div2[f], cloud_keypoints[f], cloud_keypoints[f + 1], cloud_corrs[f], cloud_smallgrid_div2_vector[f]);
		}
		else {
			splitCloudintoCloudSmallGrid2(cloud[f], cloud_smallgrid_div2[f], cloud_keypoints[f], cloud_keypoints[0], cloud_corrs[f], cloud_smallgrid_div2_vector[f]);
		}
	}
	printf("complete\n");

	//region segmentation
	bool Blood_(true);
	bool Ventricle_(true);
	bool Atrium_(true);
	bool Aorta_(true);
	bool HighPixel_(false);
	bool AtrialSeptum_(false);
	bool VentricularSeptum_(false);
	

	vector<PointCloud<PointXYZRGB>::Ptr> cloud_Blood, cloud_HighPixel, cloud_AtrialSeptum, cloud_VentricularSeptum, cloud_Ventricle, cloud_Atrium, cloud_Aorta;
	PointCloud<PointXYZRGB>::Ptr cloud_Blood_elem, cloud_HighPixel_elem, cloud_atrialSeptum_elem, cloud_VentricularSeptum_elem, cloud_Ventricle_elem, cloud_Atrium_elem, cloud_Aorta_elem;

	vector<vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>>> cloud_smallgrid_div1_blood, cloud_smallgrid_div1_highpixel;
	PointCloud<PointXYZRGB>::Ptr cloud_smallgrid_div1_blood_elem, cloud_smallgrid_div1_highpixel_elem;
	cloud_smallgrid_div1_blood.resize(cloud.size()); cloud_smallgrid_div1_highpixel.resize(cloud.size());
	vector<vector<PointCloud<PointXYZRGB>::Ptr>> cloud_clustered_before, cloud_clustered_after;
	cloud_clustered_before.resize(cloud.size()); cloud_clustered_after.resize(cloud.size());
	
	if (Blood_) {
		for (size_t i = 0; i < cloud.size(); i++) {
			constructPointXYZRGB(cloud_Blood_elem);
			cloud_Blood.push_back(cloud_Blood_elem);
		}
		for (size_t f = 0; f < cloud.size(); f++) {
			cloud_smallgrid_div1_blood[f].resize(levs / div1);
			for (int c = 0; c < levs / div1; c++) {
				cloud_smallgrid_div1_blood[f][c].resize(rows / div1);
				for (int b = 0; b < rows / div1; b++) {
					cloud_smallgrid_div1_blood[f][c][b].resize(cols / div1);
					for (int a = 0; a < cols / div1; a++) {
						constructPointXYZRGB(cloud_smallgrid_div1_blood_elem);
						cloud_smallgrid_div1_blood[f][c][b][a] = cloud_smallgrid_div1_blood_elem;
					}
				}
			}
		}
		clusteringBloodRegion2(cloud_smallgrid_div1_vector, cloud_smallgrid_div1_blood, cloud_Blood);


		//blood region clustering
		for (size_t f = 0; f < cloud.size(); f++) {
			cout << "cloud_Blood[" << f << "] found: " << cloud_Blood[f]->size() << endl;
			statisticalOutlierRemoval(cloud_Blood[f], cloud_Blood[f], OutlierK, 1.0f);
			euclideanClustering(cloud_Blood[f], 8.0f, 10, 1000, cloud_clustered_before[f]);
			recognitionBloodRegion(cloud_clustered_before[f], cloud_clustered_after[f]);
		}
		if (Ventricle_) {
			for (size_t i = 0; i < cloud.size(); i++) {
				constructPointXYZRGB(cloud_Ventricle_elem);
				cloud_Ventricle.push_back(cloud_Ventricle_elem);
			}

			clusteringLeftVentricle(timing, cloud_clustered_after[0], cloud_smallgrid_div2_vector, cloud_Ventricle);
		}
		if (Atrium_) {
			for (size_t i = 0; i < cloud.size(); i++) {
				constructPointXYZRGB(cloud_Atrium_elem);
				cloud_Atrium.push_back(cloud_Atrium_elem);
			}
			clusteringLeftAtrium(timing, cloud_clustered_after[0], cloud_smallgrid_div2_vector, cloud_Atrium);
		}
		if (Aorta_) {
			for (size_t i = 0; i < cloud.size(); i++) {
				constructPointXYZRGB(cloud_Aorta_elem);
				cloud_Aorta.push_back(cloud_Aorta_elem);
			}
			clusteringAorta(timing, cloud_clustered_after[0], cloud_smallgrid_div2_vector, cloud_Aorta);
		}
	}
	if (HighPixel_) {
		for (size_t i = 0; i < cloud.size(); i++) {
			constructPointXYZRGB(cloud_HighPixel_elem);
			cloud_HighPixel.push_back(cloud_HighPixel_elem);
		}
		for (size_t f = 0; f < cloud.size(); f++) {
			cloud_smallgrid_div1_highpixel[f].resize(levs / div1);
			for (int c = 0; c < levs / div1; c++) {
				cloud_smallgrid_div1_highpixel[f][c].resize(rows / div1);
				for (int b = 0; b < rows / div1; b++) {
					cloud_smallgrid_div1_highpixel[f][c][b].resize(cols / div1);
					for (int a = 0; a < cols / div1; a++) {
						constructPointXYZRGB(cloud_smallgrid_div1_highpixel_elem);
						cloud_smallgrid_div1_highpixel[f][c][b][a] = cloud_smallgrid_div1_highpixel_elem;
					}
				}
			}
		}
		clusteringHighPixel(cloud_smallgrid_div1_vector, cloud_smallgrid_div1_highpixel, cloud_HighPixel);
	}
	if (AtrialSeptum_) {
		for (size_t i = 0; i < cloud.size(); i++) {
			constructPointXYZRGB(cloud_atrialSeptum_elem);
			cloud_AtrialSeptum.push_back(cloud_atrialSeptum_elem);
		}
		clusteringAtrialSeptum(timing, cloud_smallgrid_div2_vector, cloud_AtrialSeptum);
	}
	if (VentricularSeptum_) {
		for (size_t i = 0; i < cloud.size(); i++) {
			constructPointXYZRGB(cloud_VentricularSeptum_elem);
			cloud_VentricularSeptum.push_back(cloud_VentricularSeptum_elem);
		}
		clusteringVentricularSeptum(timing, cloud_smallgrid_div2_vector, cloud_VentricularSeptum);
	}

	/*PointCloud<PointXYZRGB>::Ptr cloud_MitralValve(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud_AorticValve(new PointCloud<PointXYZRGB>);*/
	vector<PointCloud<PointXYZRGB>::Ptr> cloud_MitralValve, cloud_AorticValve;
	PointCloud<PointXYZRGB>::Ptr cloud_MitralValve_elem, cloud_AorticValve_elem;
	for (int i = 0; i < 8; i++) {//more than 1
		constructPointXYZRGB(cloud_MitralValve_elem); constructPointXYZRGB(cloud_AorticValve_elem);
		cloud_MitralValve.push_back(cloud_MitralValve_elem); cloud_AorticValve.push_back(cloud_AorticValve_elem);
	}

	if (Blood_) {
		locationEstimation_valve(timing, cloud_clustered_after[0], cloud_Ventricle, cloud_Atrium, cloud_Aorta, cloud, cloud_MitralValve, cloud_AorticValve);
	}

	bool valve_center_output_(true);
	if (valve_center_output_) {
		transformPointCloud(*cloud_MitralValve[0], *cloud_MitralValve[0], Eigen::Vector3f(-(cols / 2 + delta_x), -(rows / 2 + delta_y), 0), Eigen::Quaternionf(1, 0, 0, 0));
		transformPointCloud(*cloud_AorticValve[0], *cloud_AorticValve[0], Eigen::Vector3f(-(cols / 2 + delta_x), -(rows / 2 + delta_y), 0), Eigen::Quaternionf(1, 0, 0, 0));
		int index_MitralValve = cloud_MitralValve[0]->size() - 1;
		int index_AorticValve = cloud_AorticValve[0]->size() - 1;

		printf("cloud_MitralValve  x : %f  y : %f  z : %f\n", cloud_MitralValve[0]->points[index_MitralValve].x, cloud_MitralValve[0]->points[index_MitralValve].y, cloud_MitralValve[0]->points[index_MitralValve].z);
		printf("cloud_AorticValve  x : %f  y : %f  z : %f\n", cloud_AorticValve[0]->points[index_AorticValve].x, cloud_AorticValve[0]->points[index_AorticValve].y, cloud_AorticValve[0]->points[index_AorticValve].z);

	}

	//座標展開
	for (size_t f = 0; f < cloud.size(); f++) {
		modifyCloudCoordinate(cloud[f]);
		modifyCloudCoordinate(cloud_keypoints[f]);
		if (Blood_) modifyCloudCoordinate(cloud_Blood[f]);
		if (HighPixel_) modifyCloudCoordinate(cloud_HighPixel[f]);
		if (AtrialSeptum_) modifyCloudCoordinate(cloud_AtrialSeptum[f]);
		if (VentricularSeptum_) modifyCloudCoordinate(cloud_VentricularSeptum[f]);
		if (Ventricle_) modifyCloudCoordinate(cloud_Ventricle[f]);
		if (Atrium_) modifyCloudCoordinate(cloud_Atrium[f]);
		if (Aorta_) modifyCloudCoordinate(cloud_Aorta[f]);

		if (Blood_) {
			transformPointCloud(*cloud_MitralValve[f], *cloud_MitralValve[f], Eigen::Vector3f(-300, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
			transformPointCloud(*cloud_AorticValve[f], *cloud_AorticValve[f], Eigen::Vector3f(-200, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
			modifyCloudCoordinate(cloud_MitralValve[f]); modifyCloudCoordinate(cloud_AorticValve[f]);
		}

		//Blood clustered
		for (size_t i = 0; i < cloud_clustered_after[f].size(); i++) {
			modifyCloudCoordinate(cloud_clustered_after[f][i]);
		}

		for (int k = 0; k < levs / div1; k++) {
			for (int j = 0; j < rows / div1; j++) {
				for (int i = 0; i < cols / div1; i++) {
					modifyPointNormalCoordinate(cloud_smallgrid_div1_vector[f][k][j][i]);
				}
			}
		}
	}


	// Display 4D Data
	visualization::PCLVisualizer *viewer(new visualization::PCLVisualizer("4D Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	int current = 0, next = 1;
	bool show_points_(true);
	bool show_keypoints_(false);
	bool show_correspondences_(false);
	bool show_splitedCloud_Line_(false);

	//info
	bool show_info_(false);

	int count = 0;

	PointXYZRGB p0;
	p0.x = 0; p0.y = 0; p0.z = 0;

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
			bool Ventricular_only_(false);
			bool AtrialSeptum_only_(false);

			for (size_t i = 0; i < cloud_corrs[current].size(); i += 1) {
				stringstream ss_line;
				ss_line << "correspondence_line" << i;

				PointXYZRGB cloudA_point = cloud_keypoints[current]->at(cloud_corrs[current][i].index_match);
				PointXYZRGB cloudB_point = cloud_keypoints[next]->at(cloud_corrs[current][i].index_query);

				/*cloudB_point.x = cloudB_point.x - vector_ave.x;
				cloudB_point.y = cloudB_point.y - vector_ave.y;
				cloudB_point.z = cloudB_point.z - vector_ave.z;*/

				//  drawing a line for each pair of clustered correspondences found between the model and the scene
				PointXYZRGB center_point;
				center_point.x = (cols / 2.0 + delta_x)*PixelSpacingX; center_point.y = (rows / 2.0 + delta_y)*PixelSpacingY; center_point.z = 0;

				if (Ventricular_only_) {
					int deltaX = 50;
					int deltaZ = 50;
					if (cloudA_point.z / PixelSpacingZ < (levs + deltaZ)*(1 - 2 * (cloudA_point.x / PixelSpacingX) / (cols + 2 * deltaX))) {
						if ((cloudB_point.x - cloudA_point.x) >= 0 && (cloudB_point.y - cloudA_point.y) >= 0 && (cloudB_point.z - cloudA_point.z) >= 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(cloudA_point, cloudB_point, 1.0, 0, 0, ss_line.str());
						else if ((cloudB_point.x - cloudA_point.x) >= 0 && (cloudB_point.y - cloudA_point.y) < 0 && (cloudB_point.z - cloudA_point.z) >= 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(cloudA_point, cloudB_point, 1.0, 1.0, 0, ss_line.str());
						else if ((cloudB_point.x - cloudA_point.x) < 0 && (cloudB_point.y - cloudA_point.y) >= 0 && (cloudB_point.z - cloudA_point.z) >= 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(cloudA_point, cloudB_point, 0.5, 1.0, 1.0, ss_line.str());
						else if ((cloudB_point.x - cloudA_point.x) < 0 && (cloudB_point.y - cloudA_point.y) < 0 && (cloudB_point.z - cloudA_point.z) >= 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(cloudA_point, cloudB_point, 0.5, 0.5, 1.0, ss_line.str());
						else if ((cloudB_point.x - cloudA_point.x) >= 0 && (cloudB_point.y - cloudA_point.y) >= 0 && (cloudB_point.z - cloudA_point.z) < 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(cloudA_point, cloudB_point, 1.0, 0.5, 0.5, ss_line.str());
						else if ((cloudB_point.x - cloudA_point.x) >= 0 && (cloudB_point.y - cloudA_point.y) < 0 && (cloudB_point.z - cloudA_point.z) < 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(cloudA_point, cloudB_point, 1.0, 1.0, 0.5, ss_line.str());
						else if ((cloudB_point.x - cloudA_point.x) < 0 && (cloudB_point.y - cloudA_point.y) >= 0 && (cloudB_point.z - cloudA_point.z) < 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(cloudA_point, cloudB_point, 0, 1.0, 1.0, ss_line.str());
						else if ((cloudB_point.x - cloudA_point.x) < 0 && (cloudB_point.y - cloudA_point.y) < 0 && (cloudB_point.z - cloudA_point.z) < 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(cloudA_point, cloudB_point, 0, 0, 1.0, ss_line.str());
						else;
					}
				}
				else {
					//if (euclideanDistance(center_point, cloudA_point) < 185 * PixelSpacingZ) {
						//viewer->addLine<PointXYZRGB, PointXYZRGB>(cloudA_point, cloudB_point, 0.5 + (cloudB_point.x - cloudA_point.x)*0.5 / (10.0*PixelSpacingX), 0.5 + (cloudB_point.y - cloudA_point.y)*0.5 / (10.0*PixelSpacingY), 0.5 + (cloudB_point.z - cloudA_point.z)*0.5 / (10.0*PixelSpacingZ), ss_line.str());
						if ((cloudB_point.x - cloudA_point.x) >= 0 && (cloudB_point.y - cloudA_point.y) >= 0 && (cloudB_point.z - cloudA_point.z) >= 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(cloudA_point, cloudB_point, 1.0, 0, 0, ss_line.str());
						else if ((cloudB_point.x - cloudA_point.x) >= 0 && (cloudB_point.y - cloudA_point.y) < 0 && (cloudB_point.z - cloudA_point.z) >= 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(cloudA_point, cloudB_point, 1.0, 1.0, 0, ss_line.str());
						else if ((cloudB_point.x - cloudA_point.x) < 0 && (cloudB_point.y - cloudA_point.y) >= 0 && (cloudB_point.z - cloudA_point.z) >= 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(cloudA_point, cloudB_point, 0.5, 1.0, 1.0, ss_line.str());
						else if ((cloudB_point.x - cloudA_point.x) < 0 && (cloudB_point.y - cloudA_point.y) < 0 && (cloudB_point.z - cloudA_point.z) >= 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(cloudA_point, cloudB_point, 0.5, 0.5, 1.0, ss_line.str());
						else if ((cloudB_point.x - cloudA_point.x) >= 0 && (cloudB_point.y - cloudA_point.y) >= 0 && (cloudB_point.z - cloudA_point.z) < 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(cloudA_point, cloudB_point, 1.0, 0.5, 0.5, ss_line.str());
						else if ((cloudB_point.x - cloudA_point.x) >= 0 && (cloudB_point.y - cloudA_point.y) < 0 && (cloudB_point.z - cloudA_point.z) < 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(cloudA_point, cloudB_point, 1.0, 1.0, 0.5, ss_line.str());
						else if ((cloudB_point.x - cloudA_point.x) < 0 && (cloudB_point.y - cloudA_point.y) >= 0 && (cloudB_point.z - cloudA_point.z) < 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(cloudA_point, cloudB_point, 0, 1.0, 1.0, ss_line.str());
						else if ((cloudB_point.x - cloudA_point.x) < 0 && (cloudB_point.y - cloudA_point.y) < 0 && (cloudB_point.z - cloudA_point.z) < 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(cloudA_point, cloudB_point, 0, 0, 1.0, ss_line.str());
						else;
					//}
				}
			}
			PointXYZRGB vector_ave;
			vector_ave.x = 0;
			vector_ave.y = 0;
			vector_ave.z = 0;

			for (size_t i = 0; i < cloud_corrs[current].size(); i += 1) {

				PointXYZRGB& cloudP_point = cloud_keypoints[current]->at(cloud_corrs[current][i].index_match);
				PointXYZRGB& cloudN_point = cloud_keypoints[next]->at(cloud_corrs[current][i].index_query);

				vector_ave.x += cloudN_point.x - cloudP_point.x;
				vector_ave.y += cloudN_point.y - cloudP_point.y;
				vector_ave.z += cloudN_point.z - cloudP_point.z;
			}
			vector_ave.x = vector_ave.x / cloud_corrs[current].size();
			vector_ave.y = vector_ave.y / cloud_corrs[current].size();
			vector_ave.z = vector_ave.z / cloud_corrs[current].size();
			viewer->addLine<PointXYZRGB, PointXYZRGB>(p0, vector_ave, 0.5 + vector_ave.x*0.5 / (10.0*PixelSpacingX), 0.5 + vector_ave.y*0.5 / (10.0*PixelSpacingY), 0.5 + vector_ave.z*0.5 / (10.0*PixelSpacingZ), "vector_ave");
		}
		if (show_splitedCloud_Line_) {
			for (int Z = 0; Z < levs / div1; Z++) {
				for (int Y = 0; Y < rows / div1; Y++) {
					for (int X = 0; X < cols / div1; X++) {
						stringstream ss_line;
						ss_line << "split_line" << Z*rows*cols + Y*cols + X;

						int index_ave = cloud_smallgrid_div1_vector[current][Z][Y][X]->size() - 1;
						float vx_ave = cloud_smallgrid_div1_vector[current][Z][Y][X]->points[index_ave].normal_x;
						float vy_ave = cloud_smallgrid_div1_vector[current][Z][Y][X]->points[index_ave].normal_y;
						float vz_ave = cloud_smallgrid_div1_vector[current][Z][Y][X]->points[index_ave].normal_z;
						PointXYZRGB Point_current, Point_next;
						Point_current.x = cloud_smallgrid_div1_vector[current][Z][Y][X]->points[index_ave].x;
						Point_current.y = cloud_smallgrid_div1_vector[current][Z][Y][X]->points[index_ave].y;
						Point_current.z = cloud_smallgrid_div1_vector[current][Z][Y][X]->points[index_ave].z;
						Point_next.x = Point_current.x + vx_ave;
						Point_next.y = Point_current.y + vy_ave;
						Point_next.z = Point_current.z + vz_ave;


						if (vx_ave >= 0 && vy_ave >= 0 && vz_ave >= 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(Point_next, Point_current, 1.0, 0, 0, ss_line.str());
						else if (vx_ave >= 0 && vy_ave < 0 && vz_ave >= 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(Point_next, Point_current, 1.0, 1.0, 0, ss_line.str());
						else if (vx_ave < 0 && vy_ave >= 0 && vz_ave >= 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(Point_next, Point_current, 0.5, 1.0, 1.0, ss_line.str());
						else if (vx_ave < 0 && vy_ave < 0 && vz_ave >= 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(Point_next, Point_current, 0.5, 0.5, 1.0, ss_line.str());
						else if (vx_ave >= 0 && vy_ave >= 0 && vz_ave < 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(Point_next, Point_current, 1.0, 0.5, 0.5, ss_line.str());
						else if (vx_ave >= 0 && vy_ave < 0 && vz_ave < 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(Point_next, Point_current, 1.0, 1.0, 0.5, ss_line.str());
						else if (vx_ave < 0 && vy_ave >= 0 && vz_ave < 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(Point_next, Point_current, 0, 1.0, 1.0, ss_line.str());
						else if (vx_ave < 0 && vy_ave < 0 && vz_ave < 0) viewer->addLine<PointXYZRGB, PointXYZRGB>(Point_next, Point_current, 0, 0, 1.0, ss_line.str());
						else;

					}
				}
			}
		}
		if (Blood_) {
			bool Blood_cluster_(true);
			if (Blood_cluster_) {
				/*for (size_t l = 0; l < cloud_clustered_after[current].size(); l++) {
					stringstream ss;
					ss << "cloud_Blood_clustered_" << l;

					viewer->addPointCloud(cloud_clustered_after[current][l], ss.str());
					viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, ss.str());
				}*/
				viewer->addPointCloud(cloud_clustered_after[current][0], "Ventricular_Blood");
				viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 10, "Ventricular_Blood");
				viewer->addPointCloud(cloud_clustered_after[current][1], "Atrial_Blood");
				viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 7.5, "Atrial_Blood");
				viewer->addPointCloud(cloud_clustered_after[current][2], "Aortic_Blood");
				viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Aortic_Blood");
			}
			else {
				viewer->addPointCloud(cloud_Blood[current], "cloud_Blood");
				viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_Blood");
			}
		}
		if (HighPixel_) {
			viewer->addPointCloud(cloud_HighPixel[current], "cloud_HighPixel");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_HighPixel");
		}
		if (AtrialSeptum_) {
			viewer->addPointCloud(cloud_AtrialSeptum[current], "cloud_AtrialSeptum");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_AtrialSeptum");
		}
		if (VentricularSeptum_) {
			viewer->addPointCloud(cloud_VentricularSeptum[current], "cloud_VentricularSeptum");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_VentricularSeptum");
		}
		if (Ventricle_) {
			viewer->addPointCloud(cloud_Ventricle[current], "cloud_Ventricle");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_Ventricle");
		}
		if (Atrium_) {
			viewer->addPointCloud(cloud_Atrium[current], "cloud_Atrium");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_Atrium");
		}
		if (Aorta_) {
			viewer->addPointCloud(cloud_Aorta[current], "cloud_Aorta");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_Aorta");
		}
		if (Blood_) {
			viewer->addPointCloud(cloud_MitralValve[current], "cloud_MitralValve");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_MitralValve");
			viewer->addPointCloud(cloud_AorticValve[current], "cloud_AorticValve");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_AorticValve");
		}

		
		char text[16];
		sprintf_s(text, "current:%d", current);
		viewer->addText(text, 100, 0, "current");

		if (show_info_) {
			int Z, Y, X;
			if (current == 0 && count == 0) {
				cout << "Input number of Z,Y,X" << endl;
				cout << "Z = "; cin >> Z;
				cout << "Y = "; cin >> Y;
				cout << "X = "; cin >> X;
				count= 2;
			}

			displayGridInfo(viewer, cloud_smallgrid_div1_vector[current], Z, Y, X);
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