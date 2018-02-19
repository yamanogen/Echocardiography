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
	for (int i = 0; i < 9; i++) {//more than 1
		constructPointXYZRGB(cloud_elem);
		cloudMake(cloud_elem, pixeldata, i + 1);
		cloud.push_back(cloud_elem);
	}
	cout << "cloud.size() : " << cloud.size() << endl;

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

	//convert Histgram
	for (size_t i = 0; i < cloud.size(); i++) {
		convertHistgram(cloud[i], cloud[i]);
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
			uniformSampling(cloud[i], D * 2, cloud_keypoints_elem);
			cloud_keypoints.push_back(cloud_keypoints_elem);
		}
	}

	// DownSampling
	for (size_t i = 0; i < cloud.size(); i++) {
		uniformSampling(cloud_keypoints[i], D*2, cloud_keypoints[i]);
	}

	// Compute SHOT
	vector<PointCloud<SHOT352>::Ptr> cloud_descriptors;
	PointCloud<SHOT352>::Ptr cloud_descriptors_elem;
	for (size_t i = 0; i < cloud.size(); i++) {
		constructSHOT352(cloud_descriptors_elem);
		calculateSHOTOMP(cloud[i], cloud_normals[i], cloud_keypoints[i], cloud_descriptors_elem);
		cloud_descriptors.push_back(cloud_descriptors_elem);
	}

	//correspondences
	vector<vector<Correspondence>> cloud_corrs_forward;
	cloud_corrs_forward.resize(cloud.size());
	for (size_t i = 0; i < cloud.size(); i++) {
		if (i != (cloud.size() - 1)) matchDescriptors(cloud_keypoints[i], cloud_keypoints[i + 1], cloud_descriptors[i], cloud_descriptors[i + 1], cloud_corrs_forward[i]);
		else matchDescriptors(cloud_keypoints[i], cloud_keypoints[0], cloud_descriptors[i], cloud_descriptors[0], cloud_corrs_forward[i]);
	}

	cout << "Correspondences found: " << cloud_corrs_forward[0].size() << endl;

	vector<vector<Correspondence>> cloud_corrs_reverse;
	cloud_corrs_reverse.resize(cloud.size());
	for (size_t i = 0; i < cloud.size(); i++) {
		if (i != (cloud.size() - 1)) matchDescriptors(cloud_keypoints[i + 1], cloud_keypoints[i], cloud_descriptors[i + 1], cloud_descriptors[i], cloud_corrs_reverse[i]);
		else matchDescriptors(cloud_keypoints[0], cloud_keypoints[i], cloud_descriptors[0], cloud_descriptors[i], cloud_corrs_reverse[i]);
	}

	cout << "Correspondences_reverse found: " << cloud_corrs_reverse[0].size() << endl;

	vector<vector<Correspondence>> cloud_corrs;
	cloud_corrs.resize(cloud.size());
	for (size_t i = 0; i < cloud.size(); i++) {
		reversibleDescriptorMatching(cloud_corrs_forward[i], cloud_corrs_reverse[i], cloud_corrs[i]);
	}


	////storage generated vectors
	//for (size_t i = 0; i < cloud.size(); i++) {
	//	if (i != (cloud.size() - 1)) vectorOutCSV(cloud_corrs[i], cloud_keypoints[i], cloud_keypoints[i + 1], i);
	//	else vectorOutCSV(cloud_corrs[i], cloud_keypoints[i], cloud_keypoints[0], i);
	//}

	//split cloud from color
	vector<vector<PointCloud<PointXYZRGB>::Ptr>> cloud_colored;
	cloud_colored.resize(cloud.size());
	PointCloud<PointXYZRGB>::Ptr cloud_colored_elem;
	for (size_t i = 0; i < cloud.size(); i++) {//more than 1
		for (size_t j = 0; j < 8; j++) {
			constructPointXYZRGB(cloud_colored_elem);
			cloud_colored[i].push_back(cloud_colored_elem);
		}
		if (i != (cloud.size() - 1)) {
			splitCloudfromColor(cloud_keypoints[i], cloud_keypoints[i + 1], cloud_corrs[i], cloud_colored[i]);
		}
		else {
			splitCloudfromColor(cloud_keypoints[i], cloud_keypoints[0], cloud_corrs[i], cloud_colored[i]);
		}
	}

	CSVout_numberColoredCloud(cloud_colored);

	//座標展開
	for (size_t i = 0; i < cloud.size(); i++) {
		modifyCloudCoordinate(cloud[i]);
		modifyCloudCoordinate(cloud_keypoints[i]);
	}
	for (size_t i = 0; i < cloud.size(); i++) {
		for (size_t j = 0; j < 8; j++) {
			modifyCloudCoordinate(cloud_colored[i][j]);
		}
	}

	// Display 4D Data
	visualization::PCLVisualizer *viewer(new visualization::PCLVisualizer("4D Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	int current = 0, next = 1;
	int color = 0;
	bool show_points_(false);
	bool show_keypoints_(false);
	bool show_correspondences_(false);
	bool show_splitedCloud_(true);

	PointXYZRGB p0;
	p0.x = 0; p0.y = 0; p0.z = 0;

	while (!viewer->wasStopped()) {
		printf("current : %d\n", current);
		viewer->spinOnce(TimeDelay);
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

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

		/*cerr << "vector_ave : " << sqrt(pow(vector_ave.x, 2) + pow(vector_ave.y, 2) + pow(vector_ave.z, 2)) << endl;
		cerr << "cloud_corrs : " << cloud_corrs[current].size() << endl;*/

		if (show_points_) {
			viewer->addPointCloud(cloud[current], "cloud");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
		}
		if (show_keypoints_) {
			viewer->addPointCloud(cloud_keypoints[current], "cloud_keypoints");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_keypoints");
		}
		if (show_correspondences_) {
			for (size_t i = 0; i < cloud_corrs[current].size(); i += 1) {
				stringstream ss_line;
				ss_line << "correspondence_line" << i;

				PointXYZRGB cloudA_point = cloud_keypoints[current]->at(cloud_corrs[current][i].index_match);
				PointXYZRGB cloudB_point = cloud_keypoints[next]->at(cloud_corrs[current][i].index_query);

				/*cloudB_point.x = cloudB_point.x - vector_ave.x;
				cloudB_point.y = cloudB_point.y - vector_ave.y;
				cloudB_point.z = cloudB_point.z - vector_ave.z;*/

				//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
				PointXYZRGB center_point;
				center_point.x = 88; center_point.y = 70; center_point.z = 0;

				if (euclideanDistance(center_point, cloudA_point) < 185) {
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
					//viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH, 10, ss_line.str());
					/*if ((cloudB_point.x - cloudA_point.x) >= 0 && (cloudB_point.y - cloudA_point.y) >= 0 && (cloudB_point.z - cloudA_point.z) >= 0) {
					cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).r = 0; cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).g = 0; cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).b = 255;
					}
					else if ((cloudB_point.x - cloudA_point.x) >= 0 && (cloudB_point.y - cloudA_point.y) < 0 && (cloudB_point.z - cloudA_point.z) >= 0) {
					cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).r = 0; cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).g = 255; cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).b = 255;
					}
					else if ((cloudB_point.x - cloudA_point.x) < 0 && (cloudB_point.y - cloudA_point.y) >= 0 && (cloudB_point.z - cloudA_point.z) >= 0) {
					cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).r = 255; cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).g = 255; cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).b = 120;
					}
					else if ((cloudB_point.x - cloudA_point.x) < 0 && (cloudB_point.y - cloudA_point.y) < 0 && (cloudB_point.z - cloudA_point.z) >= 0) {
					cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).r = 255; cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).g = 120; cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).b = 120;
					}
					else if ((cloudB_point.x - cloudA_point.x) >= 0 && (cloudB_point.y - cloudA_point.y) >= 0 && (cloudB_point.z - cloudA_point.z) < 0) {
					cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).r = 120; cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).g = 120; cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).b = 255;
					}
					else if ((cloudB_point.x - cloudA_point.x) >= 0 && (cloudB_point.y - cloudA_point.y) < 0 && (cloudB_point.z - cloudA_point.z) < 0) {
					cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).r = 120; cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).g = 255; cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).b = 255;
					}
					else if ((cloudB_point.x - cloudA_point.x) < 0 && (cloudB_point.y - cloudA_point.y) >= 0 && (cloudB_point.z - cloudA_point.z) < 0) {
					cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).r = 255; cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).g = 255; cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).b = 0;
					}
					else if ((cloudB_point.x - cloudA_point.x) < 0 && (cloudB_point.y - cloudA_point.y) < 0 && (cloudB_point.z - cloudA_point.z) < 0) {
					cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).r = 255; cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).g = 0; cloud_keypoints[current]->at(cloud_corrs[current][i].index_match).b = 0;
					}
					else;*/
				}
			}
			viewer->addLine<PointXYZRGB, PointXYZRGB>(p0, vector_ave, 0.5 + vector_ave.x*0.5 / (10.0*PixelSpacingX), 0.5 + vector_ave.y*0.5 / (10.0*PixelSpacingY), 0.5 + vector_ave.z*0.5 / (10.0*PixelSpacingZ), "vector_ave");
			/*viewer->addPointCloud(cloud_keypoints[current], "cloud_keypoints");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_keypoints");*/
		}
		if (show_splitedCloud_) {
			viewer->addPointCloud(cloud_colored[current][color], "cloud_colored");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_colored");
		}

		current = next;
		if (next != (cloud.size() - 1)) {
			next++;
		}
		else {
			next = 0;
			if (color != (cloud_colored[current].size() - 1)) {
				color++;
			}
			else{
				color = 0;
			}
		}
	}

	return 0;
}