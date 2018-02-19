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

	/*vector<PointCloud<Normal>::Ptr> cloud_normals;
	PointCloud<Normal>::Ptr cloud_normals_elem;
	for (size_t i = 0; i < cloud.size(); i++) {
		constructNormal(cloud_normals_elem);
		cloud_normals.push_back(cloud_normals_elem);
	}
	for (size_t i = 0; i < cloud.size(); i++) {
		normalsEstimation(cloud[i], cloud_normals[i]);
	}*/

	// create cloud of kepoints
	bool keypoints_(true);
	vector<PointCloud<PointXYZRGB>::Ptr> cloud_keypoints;
	PointCloud<PointXYZRGB>::Ptr cloud_keypoints_elem;
	if (keypoints_) {
		bool keypoints_from_curvature_(true);
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
	}

	vector<PointCloud<Normal>::Ptr> cloud_normals;
	PointCloud<Normal>::Ptr cloud_normals_elem;
	for (size_t i = 0; i < cloud.size(); i++) {
		constructNormal(cloud_normals_elem);
		cloud_normals.push_back(cloud_normals_elem);
	}
	for (size_t i = 0; i < cloud.size(); i++) {
		normalsEstimation(cloud_keypoints[i], cloud_normals[i]);
	}

	//correspondences
	bool corrs_(true);
	vector<vector<Correspondence>> cloud_corrs;
	cloud_corrs.resize(cloud.size());
	vector<vector<Correspondence>> cloud_corrs_forward, cloud_corrs_reverse;
	cloud_corrs_forward.resize(cloud.size()); cloud_corrs_reverse.resize(cloud.size());

	if (corrs_) {
		bool Descriptor_SHOTorPFH_(false);
		if (Descriptor_SHOTorPFH_) {
			// Compute SHOT
			vector<PointCloud<SHOT352>::Ptr> cloud_descriptorsSHOT;
			PointCloud<SHOT352>::Ptr cloud_descriptorsSHOT_elem;
			for (size_t i = 0; i < cloud.size(); i++) {
				constructSHOT352(cloud_descriptorsSHOT_elem);
				calculateSHOTOMP(cloud[i], cloud_normals[i], cloud_keypoints[i], cloud_descriptorsSHOT_elem);
				cloud_descriptorsSHOT.push_back(cloud_descriptorsSHOT_elem);
			}
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
		}
		else {
			// Compute PFH
			vector<PointCloud<PFHSignature125>::Ptr> cloud_descriptorsPFH;
			PointCloud<PFHSignature125>::Ptr cloud_descriptorsPFH_elem;
			for (size_t i = 0; i < cloud.size(); i++) {
				constructPFH(cloud_descriptorsPFH_elem);
				calculatePFH(cloud_keypoints[i], cloud_normals[i], cloud_descriptorsPFH_elem, 4.0f);
				cloud_descriptorsPFH.push_back(cloud_descriptorsPFH_elem);
			}
			for (size_t i = 0; i < cloud.size(); i++) {
				if (i != (cloud.size() - 1)) {
					matchDescriptors2(cloud_keypoints[i], cloud_keypoints[i + 1], cloud_descriptorsPFH[i], cloud_descriptorsPFH[i + 1], cloud_corrs_forward[i]);
					matchDescriptors2(cloud_keypoints[i + 1], cloud_keypoints[i], cloud_descriptorsPFH[i + 1], cloud_descriptorsPFH[i], cloud_corrs_reverse[i]);
				}
				else {
					matchDescriptors2(cloud_keypoints[i], cloud_keypoints[0], cloud_descriptorsPFH[i], cloud_descriptorsPFH[0], cloud_corrs_forward[i]);
					matchDescriptors2(cloud_keypoints[0], cloud_keypoints[i], cloud_descriptorsPFH[0], cloud_descriptorsPFH[i], cloud_corrs_reverse[i]);
				}
				cout << "Correspondences_forward[" << i << "] found: " << cloud_corrs_forward[i].size() << endl;
				cout << "Correspondences_reverse[" << i << "] found: " << cloud_corrs_reverse[i].size() << endl;
			}
		}
		for (size_t i = 0; i < cloud.size(); i++) {
			reversibleDescriptorMatching(cloud_corrs_forward[i], cloud_corrs_reverse[i], cloud_corrs[i]);
			cout << "Correspondences[" << i << "] found: " << cloud_corrs[i].size() << endl;
		}
	}

	//座標展開
	for (size_t f = 0; f < cloud.size(); f++) {
		modifyCloudCoordinate(cloud[f]);
		if (keypoints_)modifyCloudCoordinate(cloud_keypoints[f]);
	}

	// Display 4D Data
	visualization::PCLVisualizer *viewer(new visualization::PCLVisualizer("4D Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	int current = 0, next = 1;
	bool show_points_(false);
	bool show_keypoints_(false);
	bool show_normals_(false);
	bool show_correspondences_(true);

	PointXYZRGB p0;
	p0.x = 0; p0.y = 0; p0.z = 0;

	while (!viewer->wasStopped()) {
		viewer->spinOnce(TimeDelay);
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		if (show_points_) {
			viewer->addPointCloud(cloud[current], "cloud");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
		}
		if (show_keypoints_) {
			viewer->addPointCloud(cloud_keypoints[current], "cloud_keypoints");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_keypoints");
		}
		if (show_normals_) {
			viewer->addPointCloudNormals<PointXYZRGB, Normal>(cloud_keypoints[current], cloud_normals[current], 2, 0.2f, "cloud_normals");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "cloud_normals");
		}
		if (corrs_) {
			if (show_correspondences_) {
				for (size_t i = 0; i < cloud_corrs[current].size(); i += 1) {
					stringstream ss_line;
					ss_line << "correspondence_line" << i;

					PointXYZRGB cloudA_point = cloud_keypoints[current]->at(cloud_corrs[current][i].index_match);
					PointXYZRGB cloudB_point = cloud_keypoints[next]->at(cloud_corrs[current][i].index_query);

					//  drawing a line for each pair of clustered correspondences found between the model and the scene
					PointXYZRGB center_point;
					center_point.x = (cols / 2.0 + delta_x)*PixelSpacingX; center_point.y = (rows / 2.0 + delta_y)*PixelSpacingY; center_point.z = 0;
					//if (euclideanDistance(center_point, cloudA_point) < 185 * PixelSpacingZ) {
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



				//平均ベクトル
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
		}

		printf("current : %d\n", current);
		char text[16];
		sprintf_s(text, "current:%d", current);
		viewer->addText(text, 100, 0, "current");

		current = next;
		if (next != (cloud.size() - 1)) {
			next++;
		}
		else {
			next = 0;
		}
	}

	return 0;
}