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
	int data_number = 13; int data_index = 1;
	int blood_outlier = 2;
	bool Blood_cluster_(true);
	bool Blood_cluster_after_(true);
	bool Valve_Location_(true);
	bool valve_display_mode_(true);
	int negaFromCloud_size = 0;
	
	

	if (dcmfile.loadFile(DCMfilenameTest13_6).bad()) {
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
	for (int i = 0; i < 7; i++) {//more than 1
		constructPointXYZRGB(cloud_elem);
		cloudMake(cloud_elem, pixeldata, i + 1);
		cloud.push_back(cloud_elem);
	}

	for (size_t i = 0; i < cloud.size(); i++) {
		convertHistgram(cloud[i], cloud[i]);
	}

	for (size_t i = 0; i < cloud.size(); i++) {
		cloudPixelThRemoval(cloud[i], 30);
	}

	////外れ値処理
	//for (int n = 0; n < 2; n++) {
	//	for (size_t i = 0; i < cloud.size(); i++) {
	//		statisticalOutlierRemoval(cloud[i], cloud[i], OutlierK, OutlierTh);
	//	}
	//}

	//// DownSampling
	//for (size_t i = 0; i < cloud.size(); i++) {
	//	uniformSampling(cloud[i], D, cloud[i]);
	//}

	////gaussian smoothing
	//for (size_t i = 0; i < cloud.size(); i++) {
	//	bilateralFiltering(cloud[i], cloud[i]);
	//}

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
	for (size_t i = 0; i < cloud.size(); i++) {
		bilateralFiltering(cloud[i], cloud[i]);
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
	//timing[7].push_back(1); timing[7].push_back(3);

	//split cloud into cloud_smallgrid_div1 div1=4
	vector<vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>>> cloud_smallgrid_div1;
	PointCloud<PointXYZRGB>::Ptr cloud_smallgrid_div1_elem;
	cloud_smallgrid_div1.resize(cloud.size());

	vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> cloud_smallgrid_div1_vector;
	PointCloud<PointXYZRGBNormal>::Ptr cloud_smallgrid_div1_vector_elem;
	cloud_smallgrid_div1_vector.resize(cloud.size());

	for (size_t f = 0; f < cloud.size(); f++) {
		cloud_smallgrid_div1[f].resize(levs / div1);
		for (int k = 0; k < levs / div1; k++) {
			cloud_smallgrid_div1[f][k].resize(rows / div1);
			for (int j = 0; j < rows / div1; j++) {
				cloud_smallgrid_div1[f][k][j].resize(cols / div1);
				for (int i = 0; i < cols / div1; i++) {
					constructPointXYZRGB(cloud_smallgrid_div1_elem);
					cloud_smallgrid_div1[f][k][j][i] = cloud_smallgrid_div1_elem;
				}
			}
		}
		if (f != (cloud.size() - 1)) {
			splitCloudintoCloudSmallGridFastDebug(cloud[f], cloud_smallgrid_div1[f]);
		}
		else {
			splitCloudintoCloudSmallGridFastDebug(cloud[f], cloud_smallgrid_div1[f]);
		}
	}

	//region segmentation
	/*-----------------------------------------------------*/
	bool Blood_(true);
	//bool Valve_Location_(true);
	/*-----------------------------------------------------*/

	vector<PointCloud<PointXYZRGB>::Ptr> cloud_Blood;
	PointCloud<PointXYZRGB>::Ptr cloud_Blood_elem;

	vector<vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>>> cloud_smallgrid_div1_blood;
	PointCloud<PointXYZRGB>::Ptr cloud_smallgrid_div1_blood_elem;
	cloud_smallgrid_div1_blood.resize(cloud.size());
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
		//cloud.size() - 0
		clusteringBloodRegionFastDebug(cloud_smallgrid_div1, cloud_smallgrid_div1_blood, cloud_Blood, cloud.size() - negaFromCloud_size);

		for (size_t f = 0; f < cloud.size(); f++) {
			cout << "cloud_Blood[" << f << "] found: " << cloud_Blood[f]->size() << endl;
		}
		//blood region clustering
		for (size_t f = 0; f < cloud.size(); f++) {
			cout << "cloud_Blood[" << f << "] found: " << cloud_Blood[f]->size() << endl;

			//statisticalOutlierRemoval(cloud_Blood[f], cloud_Blood[f], OutlierK, 1.0f);
			//OutlierTh Mani1~3:0.5 Mani4~6: 7~9: 10~13:11は1.2f 12は0.5f 14~16:
			/*--------------------------------------------------------------------------------------*/
			for (int i = 0; i < blood_outlier; i++) {
				statisticalOutlierRemoval(cloud_Blood[f], cloud_Blood[f], OutlierK, 0.5f);
			}
			if (Blood_cluster_) {
				euclideanClustering(cloud_Blood[f], 8.0f, 3, 10000, cloud_clustered_before[f]);
				if(Blood_cluster_after_) recognitionBloodRegion2(cloud_clustered_before[f], cloud_clustered_after[f]);
			}
		}
	}

	vector<PointCloud<PointXYZRGB>::Ptr> cloud_MitralValve, cloud_AorticValve;
	PointCloud<PointXYZRGB>::Ptr cloud_MitralValve_elem, cloud_AorticValve_elem;
	for (int i = 0; i < cloud.size(); i++) {//more than 1
		constructPointXYZRGB(cloud_MitralValve_elem); constructPointXYZRGB(cloud_AorticValve_elem);
		cloud_MitralValve.push_back(cloud_MitralValve_elem); cloud_AorticValve.push_back(cloud_AorticValve_elem);
	}

	if (Valve_Location_) {
		locationEstimation_valve2(timing, cloud_clustered_after[0], cloud, cloud_MitralValve, cloud_AorticValve);
/*--------------------------------------------------------------------------------------*/
		CSVoutMitralValveCentroid(cloud_MitralValve[0], data_number, data_index);
		CSVoutAorticValveCentroid(cloud_AorticValve[0], data_number, data_index);
/*--------------------------------------------------------------------------------------*/
		/*int index_MitralValve = cloud_MitralValve[0]->size() - 1;
		printf("cloud_MitralValve  x : %f  y : %f  z : %f\n", cloud_MitralValve[0]->points[index_MitralValve].x, cloud_MitralValve[0]->points[index_MitralValve].y, cloud_MitralValve[0]->points[index_MitralValve].z);
		int index_AorticValve = cloud_AorticValve[0]->size() - 1;
		printf("cloud_AorticValve  x : %f  y : %f  z : %f\n", cloud_AorticValve[0]->points[index_AorticValve].x, cloud_AorticValve[0]->points[index_AorticValve].y, cloud_AorticValve[0]->points[index_AorticValve].z);*/
	}

	//座標展開
	for (size_t f = 0; f < cloud.size(); f++) {
		modifyCloudCoordinate(cloud[f]);
		if (Blood_) modifyCloudCoordinate(cloud_Blood[f]);
		if (Valve_Location_) {
			/*transformPointCloud(*cloud_MitralValve[f], *cloud_MitralValve[f], Eigen::Vector3f(-300, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
			transformPointCloud(*cloud_AorticValve[f], *cloud_AorticValve[f], Eigen::Vector3f(-200, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));*/
			modifyCloudCoordinate(cloud_MitralValve[f]); modifyCloudCoordinate(cloud_AorticValve[f]);
		}

		//Blood clustered
		for (size_t i = 0; i < cloud_clustered_before[f].size(); i++) {
			modifyCloudCoordinate(cloud_clustered_before[f][i]);
		}
		for (size_t i = 0; i < cloud_clustered_after[f].size(); i++) {
			modifyCloudCoordinate(cloud_clustered_after[f][i]);
		}
	}


	// Display 4D Data
	visualization::PCLVisualizer *viewer(new visualization::PCLVisualizer("4D Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	int current = 0, next = 1;
	bool show_points_(true);

	PointXYZRGB p0;
	p0.x = 0; p0.y = 0; p0.z = 0;
	PointXYZ probe;
	probe.x = (cols / 2 + delta_x)*PixelSpacingX; probe.y = (rows / 2 + delta_y)*PixelSpacingY; probe.z = 0;
	PointCloud<PointXYZ>::Ptr Probe(new PointCloud<PointXYZ>);
	Probe->push_back(probe);
	//sinnti
	PointCloud<PointXYZRGB>::Ptr trueValve(new PointCloud<PointXYZRGB>);
	PointXYZRGB trueValve_mitral, trueValve_aortic;
	trueValve_mitral.x = PixelSpacingX * 140;
	trueValve_mitral.y = PixelSpacingY * 116;
	trueValve_mitral.z = PixelSpacingZ * 110;
	trueValve_aortic.x = PixelSpacingX * 176;
	trueValve_aortic.y = PixelSpacingY * 113;
	trueValve_aortic.z = PixelSpacingZ * 83;

	trueValve_mitral.r = 0;
	trueValve_mitral.g = 0;
	trueValve_mitral.b = 255;
	trueValve_aortic.r = 255;
	trueValve_aortic.g = 0;
	trueValve_aortic.b = 0;
	trueValve->push_back(trueValve_mitral);
	trueValve->push_back(trueValve_aortic);

	while (!viewer->wasStopped()) {
		//printf("current : %d\n", current);
		viewer->spinOnce(TimeDelay);
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();
		viewer->addPointCloud(Probe, "Probe");
		viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 20, "Probe");

		/*if (valve_display_mode_) {
			viewer->addPointCloud(trueValve, "trueValve");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 30, "trueValve");
		}*/

		if (show_points_) {
			viewer->addPointCloud(cloud[current], "cloud");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
		}
		if (Blood_) {
			/*--------------------------------------------------------------------------------------*/
			//bool Blood_cluster_(true);
			/*--------------------------------------------------------------------------------------*/
			if (Blood_cluster_) {
				for (size_t l = 0; l < cloud_clustered_before[current].size(); l++) {
					stringstream ss;
					ss << "cloud_Blood_clustered_" << l;

					viewer->addPointCloud(cloud_clustered_before[current][l], ss.str());
					viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, ss.str());
				}
				/*for (size_t l = 0; l < cloud_clustered_after[current].size(); l++) {
					stringstream ss;
					ss << "cloud_Blood_clustered_" << l;

					viewer->addPointCloud(cloud_clustered_after[current][l], ss.str());
					viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, ss.str());
				}*/
			    /*viewer->addPointCloud(cloud_clustered_after[current][0], "Ventricular_Blood");
				viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 10, "Ventricular_Blood");
				viewer->addPointCloud(cloud_clustered_after[current][1], "Atrial_Blood");
				viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 10, "Atrial_Blood");
				viewer->addPointCloud(cloud_clustered_after[current][2], "Aortic1_Blood");
				viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Aortic1_Blood");
				if (cloud_clustered_after[current].size() > 3) {
					viewer->addPointCloud(cloud_clustered_after[current][3], "Aortic2_Blood");
					viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Aortic2_Blood");
				}*/
			}
			else {
				viewer->addPointCloud(cloud_Blood[current], "cloud_Blood");
				viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_Blood");
			}
		}
		if (Valve_Location_) {


			/*viewer->addPointCloud(cloud_MitralValve[current], "cloud_MitralValve");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_MitralValve");
			viewer->addPointCloud(cloud_AorticValve[current], "cloud_AorticValve");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_AorticValve");*/

			PointCloud<PointXYZRGB>::Ptr point_valve_center(new PointCloud<PointXYZRGB>);
			int index_mitralvalve = cloud_MitralValve[current]->size() - 1;
			int index_aorticvalve = cloud_AorticValve[current]->size() - 1;

			PointXYZRGB mitralvalve, aorticvalve;
			mitralvalve.x = cloud_MitralValve[current]->points[index_mitralvalve].x;
			mitralvalve.y = cloud_MitralValve[current]->points[index_mitralvalve].y;
			mitralvalve.z = cloud_MitralValve[current]->points[index_mitralvalve].z;
			mitralvalve.r = 255;
			mitralvalve.g = 255;
			mitralvalve.b = 255;

			aorticvalve.x = cloud_AorticValve[current]->points[index_aorticvalve].x;
			aorticvalve.y = cloud_AorticValve[current]->points[index_aorticvalve].y;
			aorticvalve.z = cloud_AorticValve[current]->points[index_aorticvalve].z;
			aorticvalve.r = 255;
			aorticvalve.g = 255;
			aorticvalve.b = 255;

			point_valve_center->push_back(mitralvalve);
			point_valve_center->push_back(aorticvalve);

			viewer->addPointCloud(point_valve_center, "point_valve_center");
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 20, "point_valve_center");

			int centroid_LV = cloud_clustered_after[current][0]->size() - 1;
			int centroid_LA = cloud_clustered_after[current][1]->size() - 1;
			int centroid_aorta1 = cloud_clustered_after[current][2]->size() - 1;
			int centroid_aorta2;
			if (cloud_clustered_after[current].size() == 4) {
				centroid_aorta2 = cloud_clustered_after[current][3]->size() - 1;
			}

			
			PointXYZRGB PointCentroid_LV, PointCentroid_LA,PointCentroid_aorta1, PointCentroid_aorta2;
			PointCentroid_LV.x = cloud_clustered_after[current][0]->points[centroid_LV].x;
			PointCentroid_LV.y = cloud_clustered_after[current][0]->points[centroid_LV].y;
			PointCentroid_LV.z = cloud_clustered_after[current][0]->points[centroid_LV].z;
			PointCentroid_LA.x = cloud_clustered_after[current][1]->points[centroid_LA].x;
			PointCentroid_LA.y = cloud_clustered_after[current][1]->points[centroid_LA].y;
			PointCentroid_LA.z = cloud_clustered_after[current][1]->points[centroid_LA].z;
			PointCentroid_aorta1.x = cloud_clustered_after[current][2]->points[centroid_aorta1].x;
			PointCentroid_aorta1.y = cloud_clustered_after[current][2]->points[centroid_aorta1].y;
			PointCentroid_aorta1.z = cloud_clustered_after[current][2]->points[centroid_aorta1].z;
			
			//if (!valve_display_mode_) {
				viewer->addLine<PointXYZRGB, PointXYZRGB>(PointCentroid_LV, PointCentroid_LA, 1.0, 1.0, 1.0, "LV_LA");
				viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH, 20, "LV_LA");
				if (cloud_clustered_after[current].size() == 4) {
					PointCentroid_aorta2.x = cloud_clustered_after[current][3]->points[centroid_aorta2].x;
					PointCentroid_aorta2.y = cloud_clustered_after[current][3]->points[centroid_aorta2].y;
					PointCentroid_aorta2.z = cloud_clustered_after[current][3]->points[centroid_aorta2].z;
					viewer->addLine<PointXYZRGB, PointXYZRGB>(PointCentroid_aorta1, PointCentroid_aorta2, 1.0, 1.0, 1.0, "aorta1_aorta2");
					viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH, 20, "aorta1_aorta2");
				}
				else {
					viewer->addLine<PointXYZRGB, PointXYZRGB>(PointCentroid_aorta1, PointCentroid_LV, 1.0, 1.0, 1.0, "aorta1_LV");
					viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH, 20, "aorta1_LV");
				}
				/*viewer->addLine<PointXYZRGB, PointXYZRGB>(point_valve_center->points[0], trueValve->points[0], 1.0, 1.0, 1.0, "Arrow1");
				viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "Arrow1");
				viewer->addLine<PointXYZRGB, PointXYZRGB>(point_valve_center->points[1], trueValve->points[1], 1.0, 1.0, 1.0, "arrow2");
				viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH, 20, "arrow2");*/
			//}

			
		}


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