#include "stdafx.h"
#include "MyHeader.h"

//global var
int rows, cols, levs, frames;
double PixelSpacingX, PixelSpacingY, PixelSpacingZ;


void GetValueFromDCM(DcmDataset *dataset)
{
	DcmElement *elemRows, *elemCols, *elemLevs, *elemPixelSpacingX, *elemPixelSpacingY, *elemPixelSpacingZ, *elemFrames;
	Uint16 Rows, Cols;
	Uint32 Levs;
	Float64 pixelSpacingX, pixelSpacingY, pixelSpacingZ;
	char *Frames;

	if (!dataset->findAndGetElement(DcmTagKey(0x0028, 0x0010), elemRows).good()) {
		printf("Rows couldn't be aquired.\n");
	}
	elemRows->getUint16(Rows);

	if (!dataset->findAndGetElement(DcmTagKey(0x0028, 0x0011), elemCols).good()) {
		printf("Cols couldn't be aquired.\n");
	}
	elemCols->getUint16(Cols);

	if (!dataset->findAndGetElement(DcmTagKey(0x3001, 0x1001), elemLevs).good()) {
		printf("Levs couldn't be aquired.\n");
	}
	elemLevs->getUint32(Levs);

	if (!dataset->findAndGetElement(DcmTagKey(0x0018, 0x602c), elemPixelSpacingX).good()) {
		printf("PixelSpacingX couldn't be aquired.\n");
	}
	elemPixelSpacingX->getFloat64(pixelSpacingX);

	if (!dataset->findAndGetElement(DcmTagKey(0x0018, 0x602e), elemPixelSpacingY).good()) {
		printf("PixelSpacingY couldn't be aquired.\n");
	}
	elemPixelSpacingY->getFloat64(pixelSpacingY);

	if (!dataset->findAndGetElement(DcmTagKey(0x3001, 0x1003), elemPixelSpacingZ).good()) {
		printf("PixelSpacingZ couldn't be aquired.\n");
	}
	elemPixelSpacingZ->getFloat64(pixelSpacingZ);

	if (!dataset->findAndGetElement(DcmTagKey(0x0028, 0x0008), elemFrames).good()) {
		printf("Frames couldn't be aquired.\n");
	}
	elemFrames->getString(Frames);

	rows = (int)Rows;
	cols = (int)Cols;
	levs = (int)Levs;
	PixelSpacingX = (double)pixelSpacingX;
	PixelSpacingY = (double)pixelSpacingY;
	PixelSpacingZ = (double)pixelSpacingZ;
	frames = atoi(Frames);
}

void constructPointXYZRGB(PointCloud<PointXYZRGB>::Ptr &cloud)
{
	PointCloud<PointXYZRGB>::Ptr cloud_elem(new PointCloud<PointXYZRGB>);
	cloud = cloud_elem;
}

void constructPointXYZRGBNormal(PointCloud<PointXYZRGBNormal>::Ptr &cloud)
{
	PointCloud<PointXYZRGBNormal>::Ptr cloud_elem(new PointCloud<PointXYZRGBNormal>);
	cloud = cloud_elem;
}

void constructNormal(PointCloud<Normal>::Ptr &cloud_normals)
{
	PointCloud<Normal>::Ptr cloud_normals_elem(new PointCloud<Normal>);
	cloud_normals = cloud_normals_elem;
}

void constructPointNormal(PointCloud<PointNormal>::Ptr &cloud_PointNormal)
{
	PointCloud<PointNormal>::Ptr cloud_PointNormal_elem(new PointCloud<PointNormal>);
	cloud_PointNormal = cloud_PointNormal_elem;
}

void createPointNormal(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointNormal>::Ptr &cloud_PointNormal)
{
	cloud_PointNormal->resize(cloud->size());
	for (size_t i = 0; i < cloud->size(); i++) {
		cloud_PointNormal->points[i].x = cloud->points[i].x;
		cloud_PointNormal->points[i].y = cloud->points[i].y;
		cloud_PointNormal->points[i].z = cloud->points[i].z;
	}
}

void constructSHOT352(PointCloud<SHOT352>::Ptr &cloud_descriptors)
{
	PointCloud<SHOT352>::Ptr cloud_descriptors_elem(new PointCloud<SHOT352>);
	cloud_descriptors = cloud_descriptors_elem;
}

void constructPFH(PointCloud<PFHSignature125>::Ptr &cloud_descriptors)
{
	PointCloud<PFHSignature125>::Ptr cloud_descriptors_elem(new PointCloud<PFHSignature125>);
	cloud_descriptors = cloud_descriptors_elem;
}

void cloudMake(PointCloud<PointXYZRGB>::Ptr cloud, Uint8 *pixeldata, int n)
{
	cloud->width = cols;
	cloud->height = rows;
	cloud->is_dense = true;
	cloud->points.resize(cloud->width*cloud->height);

	for (int Z = 0; Z < levs; Z++) {
		Mat frame = Mat(Size(cols, rows), CV_8UC1, pixeldata + (n - 1)*cols*rows*levs + cols*rows*Z);

		for (int X = 0; X < frame.cols; X++) {
			for (int Y = 0; Y < frame.rows; Y++) {
				int I = frame.at<unsigned char>(Y, X);

				if (I > PixelTh) {

					PointXYZRGB p;
					p.x = X;
					p.y = Y;
					p.z = Z;
					p.r = I;
					p.g = I;
					p.b = I;

					cloud->push_back(p);
				}
			}
		}
	}

	cerr << "cloud[" << n-1 << "] : " << cloud->size() << endl;
}

void cloudPixelThRemoval(PointCloud<PointXYZRGB>::Ptr &cloud, int Th)
{
	PointCloud<PointXYZRGB>::Ptr cloud_filtered(new PointCloud<PointXYZRGB>);

	for (size_t i = 0; i < cloud->size(); i++) {
		if (cloud->points[i].r > Th) {
			cloud_filtered->push_back(cloud->points[i]);
		}
	}

	cloud->clear();
	cloud = cloud_filtered;

	cerr << "cloud : " << cloud->size() << endl;
}

void cloudMakeLeft(PointCloud<PointXYZRGB>::Ptr cloud, Uint8 *pixeldata, int n)
{
	cloud->width = cols;
	cloud->height = rows;
	cloud->is_dense = true;
	cloud->points.resize(cloud->width*cloud->height);
	int count = 0;

	for (int Z = 0; Z < levs; Z++) {
		Mat frame = Mat(Size(cols, rows), CV_8UC1, pixeldata + (n - 1)*cols*rows*levs + cols*rows*Z);

		for (int Y = 0; Y < frame.rows; Y++) {
			count = 0;
			for (int X = 0; X < frame.cols; X++) {
				int I = frame.at<unsigned char>(Y, X);
				if (I > 0 && count < 30 && Z < 170) {
					PointXYZRGB p;
					p.x = X;
					p.y = Y;
					p.z = Z;
					p.r = I;
					p.g = I;
					p.b = I;
					cloud->push_back(p);

					count++;
				}
			}
		}
	}
	cerr << "cloud : " << cloud->size() << endl;
}

void normalsEstimation(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr cloud_normals)
{
	//decide keypoints from a result of curvature caluculation
	NormalEstimation<PointXYZRGB, Normal> normal_estimation;
	search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>);
	normal_estimation.setInputCloud(cloud);
	tree->setInputCloud(cloud);
	normal_estimation.setSearchMethod(tree);
	normal_estimation.setKSearch(20);
	normal_estimation.compute(*cloud_normals);

	cerr << "cloud_normals.size : " << cloud_normals->points.size() << endl;
}

void PointNormalEstimation(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointNormal>::Ptr cloud_PointNormal)
{
	//decide keypoints from a result of curvature caluculation
	NormalEstimation<PointXYZRGB, PointNormal> normal_estimation;
	search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>);
	normal_estimation.setInputCloud(cloud);
	tree->setInputCloud(cloud);
	normal_estimation.setSearchMethod(tree);
	normal_estimation.setKSearch(20);
	normal_estimation.compute(*cloud_PointNormal);

	cerr << "cloud_PointNormal.size : " << cloud_PointNormal->points.size() << endl;
}

void uniformSampling(PointCloud<PointXYZRGB>::Ptr cloud, double leafSize, PointCloud<PointXYZRGB>::Ptr cloud_filtered)
{
	UniformSampling<PointXYZRGB> uniform_sampling;
	uniform_sampling.setInputCloud(cloud);
	uniform_sampling.setRadiusSearch(leafSize);
	uniform_sampling.filter(*cloud_filtered);
	cerr << "uniformSampling_filtered : " << cloud_filtered->size() << endl;
}

void statisticalOutlierRemoval(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr cloud_filtered, int K, double T)
{
	StatisticalOutlierRemoval<PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(K);
	sor.setStddevMulThresh(T);
	sor.filter(*cloud_filtered);

	cerr << "outlier_filtered : " << cloud_filtered->size() << endl;
}

void modifyCloudCoordinate(PointCloud<PointXYZRGB>::Ptr cloud)
{
	for (size_t i = 0; i < cloud->size(); ++i) {
		cloud->points[i].x *= PixelSpacingX;
		cloud->points[i].y *= PixelSpacingY;
		cloud->points[i].z *= PixelSpacingZ;
		//cerr << i << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << endl;
	}
}

void modifyCloudACoordinate(PointCloud<PointXYZRGBA>::Ptr cloudA)
{
	for (size_t i = 0; i < cloudA->size(); ++i) {
		cloudA->points[i].x *= PixelSpacingX;
		cloudA->points[i].y *= PixelSpacingY;
		cloudA->points[i].z *= PixelSpacingZ;
		//cerr << i << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << endl;
	}
}

void modifyNormalCloudCoordinate(PointCloud<PointNormal>::Ptr cloud_PointNormal)
{
	for (size_t i = 0; i < cloud_PointNormal->size(); ++i) {
		cloud_PointNormal->points[i].x *= PixelSpacingX;
		cloud_PointNormal->points[i].y *= PixelSpacingY;
		cloud_PointNormal->points[i].z *= PixelSpacingZ;
		//cerr << i << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << endl;
	}
}

void modifyPointNormalCoordinate(PointCloud<PointXYZRGBNormal>::Ptr cloud)
{
	for (size_t i = 0; i < cloud->size(); ++i) {
		cloud->points[i].x *= PixelSpacingX;
		cloud->points[i].y *= PixelSpacingY;
		cloud->points[i].z *= PixelSpacingZ;
		cloud->points[i].normal_x *= PixelSpacingX;
		cloud->points[i].normal_y *= PixelSpacingY;
		cloud->points[i].normal_z *= PixelSpacingZ;
		//cerr << i << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << endl;
	}
}

void distributionMapCurvatures(vector<float> curvature_v)
{
	int count[10] = { 0,0,0,0,0,0,0,0,0,0 };
	float max = 0, min = 10;
	for (size_t i = 0; i < curvature_v.size(); i++) {
		if (curvature_v[i] != 0 && curvature_v[i] < border) count[0]++;
		else if (curvature_v[i] < border * 2) count[1]++;
		else if (curvature_v[i] < border * 3) count[2]++;
		else if (curvature_v[i] < border * 4) count[3]++;
		else if (curvature_v[i] < border * 5) count[4]++;
		else if (curvature_v[i] < border * 6) count[5]++;
		else if (curvature_v[i] < border * 7) count[6]++;
		else if (curvature_v[i] < border * 8) count[7]++;
		else if (curvature_v[i] < border * 9) count[8]++;
		else count[9]++;

		if (max < curvature_v[i]) {
			max = curvature_v[i];
		}
		if (min > curvature_v[i] && curvature_v[i] != 0) {
			min = curvature_v[i];
		}
	}

	cerr << "0～" << border << " : " << count[0] << endl;
	cerr << border << "～" << border * 2 << " : " << count[1] << endl;
	cerr << border * 2 << "～" << border * 3 << " : " << count[2] << endl;
	cerr << border * 3 << "～" << border * 4 << " : " << count[3] << endl;
	cerr << border * 4 << "～" << border * 5 << " : " << count[4] << endl;
	cerr << border * 5 << "～" << border * 6 << " : " << count[5] << endl;
	cerr << border * 6 << "～" << border * 7 << " : " << count[6] << endl;
	cerr << border * 7 << "～" << border * 8 << " : " << count[7] << endl;
	cerr << border * 8 << "～" << border * 9 << " : " << count[8] << endl;
	cerr << border * 9 << "～" << border * 10 << " : " << count[9] << endl;
	//cerr << border * 7 << "～" << " : " << count[2] << endl;

	cerr << "curvature_max:" << max << "   curvature_min:" << min << endl;
}

void createKeypoints(PointCloud<PointXYZRGB>::Ptr cloud, vector<float> &curvature_v, PointCloud<PointXYZRGB>::Ptr cloud_keypoints)
{
	vector<int> k_indices;
	vector<float> k_sqr_distances;
	Eigen::Vector4f planeParametersOfNormals;

	//decide keypoints from a result of curvature caluculation
	NormalEstimation<PointXYZRGB, Normal> normal_estimation;
	search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>);
	normal_estimation.setInputCloud(cloud);
	tree->setInputCloud(cloud);
	normal_estimation.setSearchMethod(tree);
	normal_estimation.setKSearch(20);

	for (size_t i = 0; i < cloud->points.size(); i++) {
		float curvature = 0;
		double r = 5.0;
		int index = i;

		tree->radiusSearch(index, r, k_indices, k_sqr_distances);
		normal_estimation.computePointNormal(*cloud, k_indices, planeParametersOfNormals, curvature);
		curvature_v.push_back(curvature);
		//printf("%f\n", curvature);

		if (curvature < 0.25) {
			PointXYZRGB p;
			p.x = cloud->points[i].x;
			p.y = cloud->points[i].y;
			p.z = cloud->points[i].z;
			p.r = cloud->points[i].r;
			p.g = cloud->points[i].g;
			p.b = cloud->points[i].b;
			cloud_keypoints->push_back(p);
		}
	}

	cout << "cloud_keypoints.size : " << cloud_keypoints->size() << endl;
}

void fastCreateKeypoints(PointCloud<PointXYZRGB>::Ptr cloud, vector<float> &curvature_v, PointCloud<PointXYZRGB>::Ptr cloud_keypoints)
{
	printf("");
}

void calculateSHOTOMP(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr cloud_normals, PointCloud<PointXYZRGB>::Ptr cloud_keypoints, PointCloud<SHOT352>::Ptr cloud_descriptors)
{
	SHOTEstimationOMP<PointXYZRGB, Normal, SHOT352> descr_est;
	float descr_rad_(30.0f);

	descr_est.setRadiusSearch(descr_rad_);
	descr_est.setInputCloud(cloud_keypoints);
	descr_est.setInputNormals(cloud_normals);
	descr_est.setSearchSurface(cloud);
	descr_est.compute(*cloud_descriptors);

	cout << "cloud_descriptors.size : " << cloud_descriptors->size() << endl;
}

void calculatePFH(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr cloud_normals, PointCloud<PFHSignature125>::Ptr cloud_descriptors, float radius)
{
	pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud(cloud);
	pfh.setInputNormals(cloud_normals);

	// Create an empty kdtree representation, and pass it to the PFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	pfh.setSearchMethod(tree);

	// Use all neighbors in a sphere of radius ○cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	pfh.setRadiusSearch(radius);

	// Compute the features
	pfh.compute(*cloud_descriptors);

	cout << "cloud_descriptors.size : " << cloud_descriptors->size() << endl;
}

void calculatePPF(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr cloud_normals, PointCloud<PPFSignature>::Ptr cloud_descriptors)
{
	//pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PPFSignature> ppf;
	//ppf.setInputCloud(cloud);
	//ppf.setInputNormals(cloud_normals);

	//// Create an empty kdtree representation, and pass it to the PFH estimation object.
	//// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	//ppf.setSearchMethod(tree);

	//// Use all neighbors in a sphere of radius ○cm
	//// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	//ppf.setRadiusSearch(2.0);

	//// Compute the features
	//ppf.compute(*cloud_descriptors);
}

void matchDescriptors(PointCloud<PointXYZRGB>::Ptr cloud1_keypoints, PointCloud<PointXYZRGB>::Ptr cloud2_keypoints, PointCloud<SHOT352>::Ptr cloud1_descriptors, PointCloud<SHOT352>::Ptr cloud2_descriptors, vector<Correspondence> &cloud_corrs)
{
	KdTreeFLANN<PointXYZRGB> radius_search;
	KdTreeFLANN<SHOT352> match_search;
	radius_search.setInputCloud(cloud2_keypoints);

	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (size_t i = 0; i < cloud1_descriptors->size(); ++i)
	{
		//int K = 10;//the number of clouds from radiusSearch 
		vector<int> radius_indices;
		vector<float> radius_sqr_dists;
		double radius = 10.0f;//the maximum transferrence

		vector<int> neigh_indices;
		vector<float> neigh_sqr_dists;
		int eval_number = 5;

		//weight
		float k1 = 0.9;//difference of shot352
		float k2 = 1 - k1;//difference of pixel

		PointCloud<SHOT352>::Ptr radiusSearched_cloudDescriptors(new PointCloud<SHOT352>());

		if (!pcl_isfinite(cloud1_descriptors->at(i).descriptor[0])) //skipping NaNs
		{
			continue;
		}

		//  restrict the number of descriptors Spherically
		int found_radiusSerched_neighs = radius_search.radiusSearch(cloud1_keypoints->at(i), radius, radius_indices, radius_sqr_dists);
		//printf("%4d  found_radiusSerched_neighs : %4d\n", i, found_radiusSerched_neighs);
		if (found_radiusSerched_neighs) {
			for (int j = 0; j < found_radiusSerched_neighs; j++) {
				radiusSearched_cloudDescriptors->push_back(cloud2_descriptors->at(radius_indices[j]));
			}
			match_search.setInputCloud(radiusSearched_cloudDescriptors);
			//  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
			if (match_search.nearestKSearch(cloud1_descriptors->at(i), eval_number, neigh_indices, neigh_sqr_dists) == eval_number && neigh_sqr_dists[0] < 0.25f) {
				int max = 0;
				for (int k = 0; k < eval_number; k++) {
					if (max < abs(cloud2_keypoints->at(radius_indices[neigh_indices[k]]).r - cloud1_keypoints->at(i).r)) {
						max = abs(cloud2_keypoints->at(radius_indices[neigh_indices[k]]).r - cloud1_keypoints->at(i).r);
					}
				}
				float eval = 0.0f, min = 1.0f;
				int l = 0;

				for (int k = 0; k < eval_number; k++) {
					eval = k1*neigh_sqr_dists[k] + k2*abs(cloud2_keypoints->at(radius_indices[neigh_indices[k]]).r - cloud1_keypoints->at(i).r) / (max*4.0);
					if (min > eval) {
						min = eval;
						l = k;

					}
				}

				Correspondence corr(radius_indices[neigh_indices[l]], static_cast<int> (i), neigh_sqr_dists[l]);
				cloud_corrs.push_back(corr);

			}
		}
	}
}

void matchDescriptors2(PointCloud<PointXYZRGB>::Ptr cloud1_keypoints, PointCloud<PointXYZRGB>::Ptr cloud2_keypoints, PointCloud<PFHSignature125>::Ptr cloud1_descriptors, PointCloud<PFHSignature125>::Ptr cloud2_descriptors, vector<Correspondence> &cloud_corrs)
{
	KdTreeFLANN<PointXYZRGB> radius_search;
	KdTreeFLANN<PFHSignature125> match_search;
	radius_search.setInputCloud(cloud2_keypoints);

	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (size_t i = 0; i < cloud1_descriptors->size(); ++i)
	{
		//int K = 10;//the number of clouds from radiusSearch 
		vector<int> radius_indices;
		vector<float> radius_sqr_dists;
		double radius = 8.0f;//the maximum transferrence

		vector<int> neigh_indices;
		vector<float> neigh_sqr_dists;
		int eval_number = 5;

		//weight
		float k1 = 0.5;//difference of shot352
		float k2 = 1 - k1;//difference of pixel

		PointCloud<PFHSignature125>::Ptr radiusSearched_cloudDescriptors(new PointCloud<PFHSignature125>());

		//if (!pcl_isfinite(cloud1_descriptors->at(i).descriptor[0])) //skipping NaNs
		//{
		//	continue;
		//}

		//  restrict the number of descriptors Spherically
		int found_radiusSerched_neighs = radius_search.radiusSearch(cloud1_keypoints->at(i), radius, radius_indices, radius_sqr_dists);
		//printf("%4d  found_radiusSerched_neighs : %4d\n", i, found_radiusSerched_neighs);
		if (found_radiusSerched_neighs) {
			for (int j = 0; j < found_radiusSerched_neighs; j++) {
				radiusSearched_cloudDescriptors->push_back(cloud2_descriptors->at(radius_indices[j]));
			}
			match_search.setInputCloud(radiusSearched_cloudDescriptors);
			//  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
			if (match_search.nearestKSearch(cloud1_descriptors->at(i), eval_number, neigh_indices, neigh_sqr_dists) == eval_number && neigh_sqr_dists[0] < 0.5f) {
				int max = 0;
				for (int k = 0; k < eval_number; k++) {
					if (max < abs(cloud2_keypoints->at(radius_indices[neigh_indices[k]]).r - cloud1_keypoints->at(i).r)) {
						max = abs(cloud2_keypoints->at(radius_indices[neigh_indices[k]]).r - cloud1_keypoints->at(i).r);
					}
				}
				float eval = 0.0f, min = 10.0f;
				int l = 0;

				for (int k = 0; k < eval_number; k++) {
					eval = k1*neigh_sqr_dists[k] + k2*abs(cloud2_keypoints->at(radius_indices[neigh_indices[k]]).r - cloud1_keypoints->at(i).r) / (max*4.0);//divide 4 because of creation 0.25f
					if (min > eval) {
						min = eval;
						l = k;

					}
				}

				Correspondence corr(radius_indices[neigh_indices[l]], static_cast<int> (i), neigh_sqr_dists[l]);
				cloud_corrs.push_back(corr);

			}
		}
	}
}

void reversibleDescriptorMatching(vector<Correspondence> &cloud_corrs_forward, vector<Correspondence> &cloud_corrs_reverse, vector<Correspondence> &cloud_corrs)
{
	int count = 1;
	for (size_t i = 0; i < cloud_corrs_forward.size(); i++) {
		for (size_t j = 0; j < cloud_corrs_reverse.size(); j++) {
			if (cloud_corrs_forward[i].index_query == cloud_corrs_reverse[j].index_match) {
				if (cloud_corrs_forward[i].index_match == cloud_corrs_reverse[j].index_query) {
					cloud_corrs.push_back(cloud_corrs_forward[i]);
					//printf("count : %d\n", count++);
				}
			}
		}
	}
}

void vectorOutCSV(vector<Correspondence> &cloud_corrs_current, PointCloud<PointXYZRGB>::Ptr cloud_current, PointCloud<PointXYZRGB>::Ptr cloud_next,int current)
{
	vector<vector<vector<int>>> vxcsv(levs, vector<vector<int>>(rows, vector<int>(cols, 0)));
	vector<vector<vector<int>>> vycsv(levs, vector<vector<int>>(rows, vector<int>(cols, 0)));
	vector<vector<vector<int>>> vzcsv(levs, vector<vector<int>>(rows, vector<int>(cols, 0)));

	for (size_t i = 0; i < cloud_corrs_current.size(); i++) {
		PointXYZRGB& cloudA_point = cloud_current->at(cloud_corrs_current[i].index_match);
		PointXYZRGB& cloudB_point = cloud_next->at(cloud_corrs_current[i].index_query);
		//printf("cloudA_point.x:%f  cloudA_point.y:%f  cloudA_point.z:%f\n", cloudA_point.x, cloudA_point.y, cloudA_point.z);
		vxcsv[(int)cloudA_point.z][(int)cloudA_point.y][(int)cloudA_point.x] = (int)(cloudB_point.x - cloudA_point.x);
		vycsv[(int)cloudA_point.z][(int)cloudA_point.y][(int)cloudA_point.x] = (int)(cloudB_point.y - cloudA_point.y);
		vzcsv[(int)cloudA_point.z][(int)cloudA_point.y][(int)cloudA_point.x] = (int)(cloudB_point.z - cloudA_point.z);
		//printf("vxcsv:%f\n", vxcsv[(int)cloudA_point.z][(int)cloudA_point.y][(int)cloudA_point.x]);
	}

	char filenameVx[256], filenameVy[256], filenameVz[256];

	for (int k = 0; k < levs; k++) {
		sprintf_s(filenameVx, "C:\\Users\\gen\\Desktop\\心臓データ\\csv\\Test\\1\\Vector\\vx\\%d_%d.csv", current + 1, k + 1);
		sprintf_s(filenameVy, "C:\\Users\\gen\\Desktop\\心臓データ\\csv\\Test\\1\\Vector\\vy\\%d_%d.csv", current + 1, k + 1);
		sprintf_s(filenameVz, "C:\\Users\\gen\\Desktop\\心臓データ\\csv\\Test\\1\\Vector\\vz\\%d_%d.csv", current + 1, k + 1);
		ofstream saveVx(filenameVx), saveVy(filenameVy), saveVz(filenameVz);
		for (int j = 0; j < rows; j++) {
			for (int i = 0; i < cols; i++) {
					char buf[4];
					snprintf(buf, 3, "%d", vxcsv[k][j][i]);
					saveVx << buf;
					snprintf(buf, 3, "%d", vycsv[k][j][i]);
					saveVy << buf;
					snprintf(buf, 3, "%d", vzcsv[k][j][i]);
					saveVz << buf;
					if (i != (cols - 1)) {
						saveVx << ',';
						saveVy << ',';
						saveVz << ',';
					}
			}
			saveVx << endl; saveVy << endl; saveVz << endl;
		}

		printf("complete:%d-%d\n", current + 1, k + 1);
	}
}

void cloudMakeRight(PointCloud<PointXYZRGB>::Ptr cloud, Uint8 *pixeldata, int n)
{
	cloud->width = cols;
	cloud->height = rows;
	cloud->is_dense = true;
	cloud->points.resize(cloud->width*cloud->height);
	int count = 0;

	for (int Z = 0; Z < levs; Z++) {
		Mat frame = Mat(Size(cols, rows), CV_8UC1, pixeldata + (n - 1)*cols*rows*levs + cols*rows*Z);

		for (int Y = 0; Y < frame.rows; Y++) {
			count = 0;
			for (int X = 0; X < frame.cols; X++) {
				int I = frame.at<unsigned char>(Y, cols - 1 - X);
				if (I > 0 && count < 10 && Z < 170) {
					PointXYZRGB p;
					p.x = PixelSpacingX*(cols - 1 - X);
					p.y = PixelSpacingY*Y;
					p.z = PixelSpacingZ*Z;
					p.r = I;
					p.g = I;
					p.b = I;
					cloud->push_back(p);

					count++;
				}
			}
		}
	}
}

void getSideAspect(Uint8 *pixeldata, int n)
{
	int count_l = 0, count_r = 0;
	Mat leftAspect = Mat::zeros(Size(rows, levs), CV_8UC1);
	Mat rightAspect = Mat::zeros(Size(rows, levs), CV_8UC1);
	for (int Z = 0; Z < levs; Z++) {
		Mat frame = Mat(Size(cols, rows), CV_8UC1, pixeldata + (n - 1)*cols*rows*levs + Z*cols*rows);

		for (int Y = 0; Y < frame.rows; Y++) {
			count_l = 0; count_r = 0;
			for (int X = 0; X < frame.cols; X++) {
				int Il = frame.at<unsigned char>(Y, X);
				int Ir = frame.at<unsigned char>(Y, (cols - 1) - X);
				if (Il > 0 && count_l < 10 && Z < 175) {
					if (Il > leftAspect.at<unsigned char>(Z, Y)) {
						leftAspect.at<unsigned char>(Z, Y) = Il;
					}
					count_l++;
				}
				if (Ir > 0 && count_r < 10 && Z < 175) {
					if (Ir > rightAspect.at<unsigned char>(Z, Y)) {
						rightAspect.at<unsigned char>(Z, Y) = Ir;
					}
					count_r++;
				}
			}
		}
		/*
		namedWindow("DICOMtoPNG", WINDOW_AUTOSIZE);
		imshow("DICOMtoPNG", frame);
		waitKey(0);
		destroyAllWindows();
		*/
	}
	namedWindow("LeftAspect", WINDOW_AUTOSIZE);
	imshow("LeftAspect", leftAspect);
	namedWindow("RightAspect", WINDOW_AUTOSIZE);
	imshow("RightAspect", rightAspect);
	waitKey(0);
	destroyAllWindows();

}

void DownSampling(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr cloud_filtered, double filterSizeX, double filterSizeY, double filterSizeZ)
{
	VoxelGrid<PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(filterSizeX, filterSizeY, filterSizeZ);
	sor.filter(*cloud_filtered);

	cerr << "downsampling_filtered: " << cloud_filtered->size() << endl;
}

void PThrough(PointCloud<PointXYZRGB>::Ptr cloud)
{
	// Build a passthrough filter to remove spurious NaNs
	PassThrough<PointXYZRGB> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(4.3, 11);
	pass.filter(*cloud);
}

void Segmentation(PointCloud<PointXYZRGB>::Ptr cloud)
{
	// All the objects needed
	PCDWriter writer;
	PassThrough<PointXYZRGB> pass;
	NormalEstimation<PointXYZRGB, Normal> ne;
	SACSegmentationFromNormals<PointXYZRGB, Normal> seg;
	ExtractIndices<PointXYZRGB> extract;
	ExtractIndices<Normal> extract_normals;
	search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>());

	// Datasets
	PointCloud<PointXYZRGB>::Ptr cloud_filtered(new PointCloud<PointXYZRGB>);
	PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>);
	PointCloud<PointXYZRGB>::Ptr cloud_filtered2(new PointCloud<PointXYZRGB>);
	PointCloud<Normal>::Ptr cloud_normals2(new PointCloud<Normal>);
	ModelCoefficients::Ptr coefficients_plane(new ModelCoefficients), coefficients_cylinder(new ModelCoefficients);
	PointIndices::Ptr inliers_plane(new PointIndices), inliers_cylinder(new PointIndices);

	cerr << "PointCloud has: " << cloud->points.size() << " data points." << endl;

	// Build a passthrough filter to remove spurious NaNs

	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, 1.5);
	pass.filter(*cloud_filtered);
	cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << endl;

	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_filtered);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	// Create the segmentation object for the planar model and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight(0.1);
	seg.setMethodType(SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.03);
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(cloud_normals);
	// Obtain the plane inliers and coefficients
	seg.segment(*inliers_plane, *coefficients_plane);
	cerr << "Plane coefficients: " << *coefficients_plane << endl;

	// Extract the planar inliers from the input cloud
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);

	// Write the planar inliers to disk
	PointCloud<PointXYZRGB>::Ptr cloud_plane(new PointCloud<PointXYZRGB>());
	extract.filter(*cloud_plane);
	cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << endl;
	writer.write("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

	// Remove the planar inliers, extract the rest
	extract.setNegative(true);
	extract.filter(*cloud_filtered2);
	extract_normals.setNegative(true);
	extract_normals.setInputCloud(cloud_normals);
	extract_normals.setIndices(inliers_plane);
	extract_normals.filter(*cloud_normals2);

	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(SACMODEL_CYLINDER);
	seg.setMethodType(SAC_RANSAC);
	seg.setNormalDistanceWeight(0.1);
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(0.05);
	seg.setRadiusLimits(0, 0.1);
	seg.setInputCloud(cloud_filtered2);
	seg.setInputNormals(cloud_normals2);

	// Obtain the cylinder inliers and coefficients
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	cerr << "Cylinder coefficients: " << *coefficients_cylinder << endl;

	// Write the cylinder inliers to disk
	extract.setInputCloud(cloud_filtered2);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	PointCloud<PointXYZRGB>::Ptr cloud_cylinder(new PointCloud<PointXYZRGB>());
	extract.filter(*cloud_cylinder);
	if (cloud_cylinder->points.empty())
		cerr << "Can't find the cylindrical component." << endl;
	else
	{
		cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << endl;
		writer.write("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
	}
}

void segmentedColorChange(PointCloud<PointXYZRGB>::Ptr cloud, PointIndices::Ptr inliers, ModelCoefficients::Ptr coefficients)
{
	if (inliers->indices.size() == 0) {
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
	}
	else {
		cerr << "Model coefficients: " << coefficients->values[0] << " "
			<< coefficients->values[1] << " "
			<< coefficients->values[2] << " "
			<< coefficients->values[3] << endl;

		cerr << "Model inliers: " << inliers->indices.size() << endl;
		for (size_t i = 0; i < inliers->indices.size(); ++i) {
			//cerr << inliers->indices[i] << "    " << cloud1->points[inliers->indices[i]].x << " " << cloud1->points[inliers->indices[i]].y << " " << cloud1->points[inliers->indices[i]].z << endl;
			cloud->points[inliers->indices[i]].r = 255;
			cloud->points[inliers->indices[i]].g = 0;
			cloud->points[inliers->indices[i]].b = 0;
		}
	}
}

void projectPlane(PointCloud<PointXYZRGB>::Ptr cloud)
{
	// Create a set of planar coefficients with X=Y=0,Z=1 or X=Z=0,Y=1 or Y=Z=0,X=1 or others
	ModelCoefficients::Ptr coefficients(new ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[1] = coefficients->values[2] = 0;
	coefficients->values[0] = 1.0;
	coefficients->values[3] = 0;

	// Create the filtering object
	ProjectInliers<PointXYZRGB> proj;
	proj.setModelType(SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud);

}

void compressCoordinate(PointCloud<PointXYZRGB>::Ptr cloud)
{
	for (size_t i = 0; i < cloud->size(); ++i) {

		if (((cloud->points[i].x / D) - (int)(cloud->points[i].x / D)) < 0.5) {
			cloud->points[i].x = (int)(cloud->points[i].x / D);
		}
		else {
			cloud->points[i].x = ((int)(cloud->points[i].x / D)) + 1;
		}
		if (((cloud->points[i].y / D) - (int)(cloud->points[i].y / D)) < 0.5) {
			cloud->points[i].y = (int)(cloud->points[i].y / D);
		}
		else {
			cloud->points[i].y = ((int)(cloud->points[i].y / D)) + 1;
		}
		if (((cloud->points[i].z / D) - (int)(cloud->points[i].z / D)) < 0.5) {
			cloud->points[i].z = (int)(cloud->points[i].z / D);
		}
		else {
			cloud->points[i].z = ((int)(cloud->points[i].z / D)) + 1;
		}
		//cerr << i << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << endl;
	}
	//cerr << "cloud1: " << cloud->size() << endl;
}

void modifyVectorCoordinate(PointCloud<PointNormal>::Ptr vector)
{
	for (size_t i = 0; i < vector->size(); ++i) {
		vector->points[i].x *= times*PixelSpacingX;
		vector->points[i].y *= times*PixelSpacingY;
		vector->points[i].z *= times*PixelSpacingZ;
		//cerr << i << "    " << vector->points[i].x << " " << vector->points[i].y << " " << vector->points[i].z << endl;
	}
}

void outputCSV(PointCloud<PointXYZRGB>::Ptr cloud, int n)
{
	cerr << "start" << endl;
	int Z, Y, X;
	Z = (int)(levs / D) + 1; Y = (int)(rows / D) + 1; X = (int)(cols / D) + 1;

	//３次元配列動的メモリ割り当て
	int ***PixelData;
	PixelData = (int***)malloc(sizeof(int**)*Z);
	for (int k = 0; k < Z; k++) {
		PixelData[k] = (int**)malloc(sizeof(int*)*Y);
	}
	for (int k = 0; k < Z; k++) {
		for (int j = 0; j < Y; j++) {
			PixelData[k][j] = (int*)malloc(sizeof(int)*X);
		}
	}
	
	for (int z = 0; z < Z; z++) {
		for (int y = 0; y < Y; y++) {
			for (int x = 0; x < X; x++) {
				PixelData[z][y][x] = 0;
				//printf("syoki\n");
			}
		}
	}

	int a, b, c = 0;
	for (size_t i = 0; i < cloud->size(); ++i) {
		a = cloud->points[i].x;
		b = cloud->points[i].y;
		c = cloud->points[i].z;
		if (PixelData[c][b][a] < cloud->points[i].r) {
			PixelData[c][b][a] = cloud->points[i].r;

			//printf("%d\n", cloud->points[i].r);
			//printf("done\n");
		}
	}

	char filename[256];
	
	for (int k = 0; k < Z; k++) {
		sprintf_s(filename, "C:\\Users\\gen\\Desktop\\心臓データ\\csv\\Goal\\2\\Outlier\\DownSampling%.1f\\Volume\\%d_%d.csv", D, n, k + 1);
		ofstream save(filename);
		for (int j = 0; j < Y; j++) {
			for (int i = 0; i < X; i++) {
				char buf[4];
				snprintf(buf, 3, "%d", PixelData[k][j][i]);
				save << buf;
				if (i != 43) save << ',';
			}
			save << endl;
		}

		printf("complete:%d_%d\n", n, k + 1);
	}
}

void vectorMake(PointCloud<PointNormal>::Ptr vector, int F)
{
	vector->width = cols;
	vector->height = rows;
	vector->is_dense = true;
	vector->points.resize(vector->width*vector->height);

	for (float Z = 0; Z < levs; Z+=D) {
		char filenameVx[256];
		char filenameVy[256];
		char filenameVz[256];
		sprintf_s(filenameVx, "C:\\Users\\gen\\Desktop\\心臓データ\\csv\\Test\\1\\DownSampling%.1f\\vx\\%d_%d.csv", D, F, (int)(Z / D) + 1);
		sprintf_s(filenameVy, "C:\\Users\\gen\\Desktop\\心臓データ\\csv\\Test\\1\\DownSampling%.1f\\vy\\%d_%d.csv", D, F, (int)(Z / D) + 1);
		sprintf_s(filenameVz, "C:\\Users\\gen\\Desktop\\心臓データ\\csv\\Test\\1\\DownSampling%.1f\\vz\\%d_%d.csv", D, F, (int)(Z / D) + 1);
		ifstream ifsVx(filenameVx); if (!ifsVx) { cout << "Can't open vx" << endl; }
		ifstream ifsVy(filenameVy); if (!ifsVy) { cout << "Can't open vy" << endl; }
		ifstream ifsVz(filenameVz); if (!ifsVz) { cout << "Can't open vz" << endl; }

		for (float Y = 0; Y < rows; Y+=D) {
			string strVx,strVy,strVz;
			if (getline(ifsVx, strVx) && getline(ifsVy, strVy) && getline(ifsVz, strVz)) {
				string tokenVx,tokenVy,tokenVz;
				istringstream streamVx(strVx), streamVy(strVy), streamVz(strVz);

				for (float X = 0; X < cols; X+=D) {
					PointNormal n;
					n.x = X;
					n.y = Y;
					n.z = Z;
					//printf("%.1f", D);
					if (getline(streamVx, tokenVx, ','))n.normal_x = stof(tokenVx); //printf("  normal_x:%.3f", n.normal_x);
					if (getline(streamVy, tokenVy, ','))n.normal_y = stof(tokenVy); //printf("  normal_y:%.3f", n.normal_y);
					if (getline(streamVz, tokenVz, ','))n.normal_z = stof(tokenVz); //printf("  normal_z:%.3f\n", n.normal_z);

					vector->push_back(n);
				}
			}
			else cout << "Can't get strV" << endl;
			
		}
	}
}

void printVector(PointCloud<PointNormal>::Ptr vector)
{
	for (size_t i = 0; i < vector->size(); ++i) {
		if (vector->points[i].normal_x) {
			printf("%d   x : %.3f", i, vector->points[i].x);
			printf("   y : %.3f", vector->points[i].y);
			printf("   z : %.3f\n", vector->points[i].z);

			printf("   vx : %.3f", vector->points[i].normal_x);
			printf("  vy : %.3f", vector->points[i].normal_y);
			printf("  vz : %.3f\n", vector->points[i].normal_z);
		}
	}
}

void addVectorLine(PointCloud<PointNormal>::Ptr vector, boost::shared_ptr<visualization::PCLVisualizer> viewer)
{
	char ID[8];
	int id = 1;
	for (size_t i = 0; i < vector->size(); i++) {
		if (vector->points[i].normal_x || vector->points[i].normal_y || vector->points[i].normal_z) {
			PointXYZ p0, p1;
			p0.x = times*vector->points[i].x;
			p0.y = times*vector->points[i].y;
			p0.z = times*vector->points[i].z;
			p1.x = p0.x + timesV*vector->points[i].normal_x;
			p1.y = p0.y + timesV*vector->points[i].normal_y;
			p1.z = p0.z + timesV*vector->points[i].normal_z;
			
			sprintf_s(ID, "%d", id);
			viewer->addLine<PointXYZ, PointXYZ>(p1, p0, 1.0, 0, 0, ID);
			id++;
		}
	}
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH, 10, ID);
}

void removeVectorLine(PointCloud<PointNormal>::Ptr vector, boost::shared_ptr<visualization::PCLVisualizer> viewer)
{
	char ID[8];
	int id = 1;
	for (size_t i = 0; i < vector->size(); i++) {
		if (vector->points[i].normal_x || vector->points[i].normal_y || vector->points[i].normal_z) {
			sprintf_s(ID, "%d", id);
			viewer->removePointCloud(ID, 0);
			id++;
		}
	}
}

void addCloud(PointCloud<PointXYZRGB>::Ptr cloud, boost::shared_ptr<visualization::PCLVisualizer> viewer)
{
	viewer->addPointCloud<PointXYZRGB>(cloud, "cloud");
	//viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	printf("display\n");
}

PointCloud<PointNormal>::Ptr Surface_normals(PointCloud<PointXYZRGB>::Ptr cloud)
{
	NormalEstimation<PointXYZRGB, PointNormal> ne;
	ne.setInputCloud(cloud);//法線の計算を行いたい点群を指定する

	search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>());//KDTREEを作る
	ne.setSearchMethod(tree);//検索方法にKDTREEを指定する

	PointCloud<PointNormal>::Ptr cloud_normals(new PointCloud<PointNormal>);//法線情報を入れる変数

	ne.setRadiusSearch(0.5);//検索する半径を指定する

	ne.compute(*cloud_normals);//法線情報の出力先を指定する

	return cloud_normals;
}

PointCloud<PointWithScale> Extract_SIFT(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointNormal>::Ptr cloud_normals)
{
	// SIFT特徴量計算のためのパラメータ
	const float min_scale = 0.01f;
	const int n_octaves = 3;
	const int n_scales_per_octave = 4;
	const float min_contrast = 0.001f;
	SIFTKeypoint<PointNormal, PointWithScale> sift;
	PointCloud<PointWithScale> result;
	search::KdTree<PointNormal>::Ptr tree(new search::KdTree<PointNormal>());
	sift.setSearchMethod(tree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(0.00);
	sift.setInputCloud(cloud_normals);
	sift.compute(result);
	cout << "No of SIFT points in the result are " << result.points.size() << endl;

	return result;
}

void setColorForEachCurvature(PointCloud<PointXYZRGB>::Ptr cloud, vector<float> curvature_v)
{
	int count = 0;
	for (size_t i = 0; i < curvature_v.size(); i++) {
		/*
		if (curvature_v[i] < 0.1 && curvature_v[i] != 0) {
			cloud->points[i].r = 255;
			cloud->points[i].g = 0;
			cloud->points[i].b = 0;
			count++;
		}
		*/
		/*
		else if (curvature_v[i] < 0.18) {
			cloud->points[i].r = 255;
			cloud->points[i].g = 255;
			cloud->points[i].b = 0;
			count++;
		}
		*/
		if (0.2>curvature_v[i]) {
			cloud->points[i].r = 255;
			cloud->points[i].g = 255;
			cloud->points[i].b = 0;
			count++;
		}
		/*
		else {
			cloud->points[i].r = 0;
			cloud->points[i].g = 0;
			cloud->points[i].b = 0;
		}
		*/
	}

	cerr << "coloredCloud.size : " << count << endl;
}

void convertHistgram(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr cloud_filtered)
{
	int max = 0;
	for (size_t i = 0; i < cloud->size(); i++) {
		if (max < cloud->points[i].r) {
			max = cloud->points[i].r;
		}
	}

	for (size_t i = 0; i < cloud->size(); i++) {
		cloud_filtered->points[i].r = 255 / (max - PixelTh)*(cloud->points[i].r - PixelTh);
		cloud_filtered->points[i].g = cloud_filtered->points[i].r;
		cloud_filtered->points[i].b = cloud_filtered->points[i].r;
	}

	cerr << "convert Histgram" << endl;
}

void Binarization(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr cloud_filtered)
{
	for (size_t i = 0; i < cloud->size(); i++) {
		if (cloud->points[i].r > 60) {
			cloud_filtered->points[i].r = 255;
			cloud_filtered->points[i].g = 255;
			cloud_filtered->points[i].b = 255;
		}
		else {
			cloud_filtered->points[i].r = 0;
			cloud_filtered->points[i].g = 0;
			cloud_filtered->points[i].b = 0;
		}
	}
}

void Threshold(PointCloud<PointXYZRGB>::Ptr cloud)
{
	for (int i = 0; i < cloud->size(); i++) {
		if (cloud->points[i].r < 30) {
			cloud->points[i].r = 0;
			cloud->points[i].g = 0;
			cloud->points[i].b = 0;
		}
	}
}

void edgeDetection(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr cloud_normals, vector<PointIndices> &label_indices)
{
	/*PointCloud<Normal>::Ptr normal(new PointCloud<Normal>);
	IntegralImageNormalEstimation<PointXYZRGB, Normal> ne;
	ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
	ne.setNormalSmoothingSize(5.0f);
	ne.setBorderPolicy(ne.BORDER_POLICY_IGNORE);
	ne.setInputCloud(cloud);
	ne.compute(*normal);*/

	OrganizedEdgeFromNormals<PointXYZRGB, Normal, Label> oed;
	//OrganizedEdgeFromRGBNormals<PointXYZRGB, Normal, Label> oed; 
	oed.setInputNormals(cloud_normals);
	oed.setInputCloud(cloud);
	oed.setDepthDisconThreshold(0.2f);
	oed.setMaxSearchNeighbors(50);
	//oed.setEdgeType(oed.EDGELABEL_RGB_CANNY);
	oed.setEdgeType(oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED | oed.EDGELABEL_HIGH_CURVATURE | oed.EDGELABEL_RGB_CANNY);
	PointCloud<Label> labels;

	oed.compute(labels, label_indices);
}

void splitCloudfromColor(PointCloud<PointXYZRGB>::Ptr cloud1_keypoints, PointCloud<PointXYZRGB>::Ptr cloud2_keypoints, vector<Correspondence> &cloud_corrs, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_colored)
{
	for (size_t i = 0; i < cloud_corrs.size(); i += 1) {
		PointXYZRGB cloudA_point = cloud1_keypoints->at(cloud_corrs[i].index_match);
		PointXYZRGB cloudB_point = cloud2_keypoints->at(cloud_corrs[i].index_query);
		//cerr << "cloud_corrs : " << i << endl;

		if ((cloudB_point.x - cloudA_point.x) >= 0 && (cloudB_point.y - cloudA_point.y) >= 0 && (cloudB_point.z - cloudA_point.z) >= 0) {
			cloud1_keypoints->at(cloud_corrs[i].index_match).r = 255; cloud1_keypoints->at(cloud_corrs[i].index_match).g = 0; cloud1_keypoints->at(cloud_corrs[i].index_match).b = 0;
			cloud_colored[0]->push_back(cloud1_keypoints->at(cloud_corrs[i].index_match));
		}
		else if ((cloudB_point.x - cloudA_point.x) >= 0 && (cloudB_point.y - cloudA_point.y) < 0 && (cloudB_point.z - cloudA_point.z) >= 0) {
			cloud1_keypoints->at(cloud_corrs[i].index_match).r = 255; cloud1_keypoints->at(cloud_corrs[i].index_match).g = 255; cloud1_keypoints->at(cloud_corrs[i].index_match).b = 0;
			cloud_colored[1]->push_back(cloud1_keypoints->at(cloud_corrs[i].index_match));
		}
		else if ((cloudB_point.x - cloudA_point.x) < 0 && (cloudB_point.y - cloudA_point.y) >= 0 && (cloudB_point.z - cloudA_point.z) >= 0) {
			cloud1_keypoints->at(cloud_corrs[i].index_match).r = 120; cloud1_keypoints->at(cloud_corrs[i].index_match).g = 255; cloud1_keypoints->at(cloud_corrs[i].index_match).b = 255;
			cloud_colored[2]->push_back(cloud1_keypoints->at(cloud_corrs[i].index_match));
		}
		else if ((cloudB_point.x - cloudA_point.x) < 0 && (cloudB_point.y - cloudA_point.y) < 0 && (cloudB_point.z - cloudA_point.z) >= 0) {
			cloud1_keypoints->at(cloud_corrs[i].index_match).r = 120; cloud1_keypoints->at(cloud_corrs[i].index_match).g = 120; cloud1_keypoints->at(cloud_corrs[i].index_match).b = 255;
			cloud_colored[3]->push_back(cloud1_keypoints->at(cloud_corrs[i].index_match));
		}
		else if ((cloudB_point.x - cloudA_point.x) >= 0 && (cloudB_point.y - cloudA_point.y) >= 0 && (cloudB_point.z - cloudA_point.z) < 0) {
			cloud1_keypoints->at(cloud_corrs[i].index_match).r = 255; cloud1_keypoints->at(cloud_corrs[i].index_match).g = 120; cloud1_keypoints->at(cloud_corrs[i].index_match).b = 120;
			cloud_colored[4]->push_back(cloud1_keypoints->at(cloud_corrs[i].index_match));
		}
		else if ((cloudB_point.x - cloudA_point.x) >= 0 && (cloudB_point.y - cloudA_point.y) < 0 && (cloudB_point.z - cloudA_point.z) < 0) {
			cloud1_keypoints->at(cloud_corrs[i].index_match).r = 255; cloud1_keypoints->at(cloud_corrs[i].index_match).g = 255; cloud1_keypoints->at(cloud_corrs[i].index_match).b = 120;
			cloud_colored[5]->push_back(cloud1_keypoints->at(cloud_corrs[i].index_match));
		}
		else if ((cloudB_point.x - cloudA_point.x) < 0 && (cloudB_point.y - cloudA_point.y) >= 0 && (cloudB_point.z - cloudA_point.z) < 0) {
			cloud1_keypoints->at(cloud_corrs[i].index_match).r = 0; cloud1_keypoints->at(cloud_corrs[i].index_match).g = 255; cloud1_keypoints->at(cloud_corrs[i].index_match).b = 255;
			cloud_colored[6]->push_back(cloud1_keypoints->at(cloud_corrs[i].index_match));
		}
		else if ((cloudB_point.x - cloudA_point.x) < 0 && (cloudB_point.y - cloudA_point.y) < 0 && (cloudB_point.z - cloudA_point.z) < 0) {
			cloud1_keypoints->at(cloud_corrs[i].index_match).r = 0; cloud1_keypoints->at(cloud_corrs[i].index_match).g = 0; cloud1_keypoints->at(cloud_corrs[i].index_match).b = 255;
			cloud_colored[7]->push_back(cloud1_keypoints->at(cloud_corrs[i].index_match));
		}
		else;
	}

	cerr << "cloud_colored[0] : " << cloud_colored[0]->size() << endl;
	cerr << "cloud_colored[1] : " << cloud_colored[1]->size() << endl;
	cerr << "cloud_colored[2] : " << cloud_colored[2]->size() << endl;
	cerr << "cloud_colored[3] : " << cloud_colored[3]->size() << endl;
	cerr << "cloud_colored[4] : " << cloud_colored[4]->size() << endl;
	cerr << "cloud_colored[5] : " << cloud_colored[5]->size() << endl;
	cerr << "cloud_colored[6] : " << cloud_colored[6]->size() << endl;
	cerr << "cloud_colored[7] : " << cloud_colored[7]->size() << endl;
}

void CSVout_numberColoredCloud(vector<vector<PointCloud<PointXYZRGB>::Ptr>> cloud_colored)
{
	char filename[256];
	sprintf_s(filename, "C:\\Users\\gen\\Desktop\\心臓データ\\csv\\Goal\\1\\numberColoredCloud\\downsampling2.0.csv");
	ofstream save(filename);

	for (size_t i = 0; i < cloud_colored.size(); i++) {
		for (size_t j = 0; j < cloud_colored[i].size(); j++) {
			char buf[5];
			snprintf(buf, 4, "%d", cloud_colored[i][j]->size());
			save << buf;
			if (j != (cloud_colored[i].size()-1)) save << ',';
		}
		save << endl;
	}
}

void CSVoutVector(PointCloud<PointXYZRGB>::Ptr cloud1_keypoints, PointCloud<PointXYZRGB>::Ptr cloud2_keypoints, vector<Correspondence> &cloud_corrs, int frame)
{
	char filename[256];
	sprintf_s(filename, "C:\\Users\\gen\\Desktop\\心臓データ\\csv\\Test\\1\\Vector\\downsampling2.0\\%d.csv", frame);
	ofstream save(filename);

	for (size_t i = 0; i < cloud_corrs.size(); i++) {
		PointXYZRGB& cloudA_point = cloud1_keypoints->at(cloud_corrs[i].index_match);
		PointXYZRGB& cloudB_point = cloud2_keypoints->at(cloud_corrs[i].index_query);
		char bufx[4], bufy[4], bufz[4];
		snprintf(bufx, 3, "%d", (int)(cloudB_point.x - cloudA_point.x)); save << bufx; save << ',';
		snprintf(bufy, 3, "%d", (int)(cloudB_point.y - cloudA_point.y)); save << bufy; save << ',';
		snprintf(bufz, 3, "%d", (int)(cloudB_point.z - cloudA_point.z)); save << bufz; save << endl;
	}
}

void CSVoutVectorAVE(PointXYZRGB vector_ave, int frame)
{
	char filename[256];
	sprintf_s(filename, "C:\\Users\\gen\\Desktop\\心臓データ\\csv\\Goal\\1\\Vector\\ave.csv", frame);
	ofstream save(filename);

	char bufx[6], bufy[6], bufz[6];
	snprintf(bufx, 5, "%.2f", vector_ave.x); save << bufx; save << ',';
	snprintf(bufy, 5, "%.2f", vector_ave.y); save << bufy; save << ',';
	snprintf(bufz, 5, "%.2f", vector_ave.z); save << bufz; save << endl;
}

void splitCloudintoCloudSmallGrid(PointCloud<PointXYZRGB>::Ptr cloud, vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>> &cloud_smallgrid, PointCloud<PointXYZRGB>::Ptr cloud1_keypoints, PointCloud<PointXYZRGB>::Ptr cloud2_keypoints, vector<Correspondence> &cloud_corrs, vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>> &cloud_smallgrid_vector)
{
	for (size_t i = 0; i < cloud->size(); i++) {
		PointXYZRGB p;
		p.x = cloud->points[i].x;
		p.y = cloud->points[i].y;
		p.z = cloud->points[i].z;
		p.r = cloud->points[i].r;
		p.g = cloud->points[i].g;
		p.b = cloud->points[i].b;

		int X = 0, Y = 0, Z = 0;
		X = (int)(p.x / div1);
		Y = (int)(p.y / div1);
		Z = (int)(p.z / div1);

		cloud_smallgrid[Z][Y][X]->push_back(p);
	}

	//calculate centroid and vector_ave
	for (int k = 0; k < levs / div1; k++) {
		for (int j = 0; j < rows / div1; j++) {
			for (int i = 0; i < cols / div1; i++) {
				size_t N = cloud_smallgrid[k][j][i]->size();
				float gx = 0, gy = 0, gz = 0, pixel_ave = 0;

				for (size_t n = 0; n < N; n++) {
					gx += cloud_smallgrid[k][j][i]->points[n].x;
					gy += cloud_smallgrid[k][j][i]->points[n].y;
					gz += cloud_smallgrid[k][j][i]->points[n].z;
					pixel_ave += (float)cloud_smallgrid[k][j][i]->points[n].r;
				}

				//cerr << "N:" << N << "  pixel_ave:" << pixel_ave << endl;

				PointXYZRGB p_ave;
				if (N != 0) {
					p_ave.x = gx / N;
					p_ave.y = gy / N;
					p_ave.z = gz / N;
					//p_ave.r = (int)(pixel_ave / pow(div1 / 2, 3));
					p_ave.r = (int)(pixel_ave / N);
					p_ave.g = p_ave.r;
					p_ave.b = p_ave.r;
				}
				else {
					p_ave.x = i * div1 + (div1 - 1) / 2.0;
					p_ave.y = j * div1 + (div1 - 1) / 2.0;
					p_ave.z = k * div1 + (div1 - 1) / 2.0;
					p_ave.r = 0;
					p_ave.g = 0;
					p_ave.b = 0;
				}
				
				//printf("cloud_smallgrid[%d][%d][%d].pix_ave:%d\n", k, j, i, p_ave.r);
				//cerr << "cloud_smallgrid[" << k << "][" << j << "][" << i << "].pix_ave:" << p_ave.r << endl;
				cloud_smallgrid[k][j][i]->push_back(p_ave);//input pointVector_ave into last number of cloud_smallgrid
			}
		}
	}


	for (size_t i = 0; i < cloud_corrs.size(); i++) {
		PointXYZRGB cloudA_point = cloud1_keypoints->at(cloud_corrs[i].index_match);
		PointXYZRGB cloudB_point = cloud2_keypoints->at(cloud_corrs[i].index_query);
		//cerr << "cloud_corrs : " << i << endl;

		PointXYZRGBNormal pointVector;
		pointVector.x = cloudA_point.x;
		pointVector.y = cloudA_point.y;
		pointVector.z = cloudA_point.z;
		pointVector.r = cloudA_point.r;
		pointVector.g = cloudA_point.g;
		pointVector.b = cloudA_point.b;
		pointVector.normal_x = cloudB_point.x - cloudA_point.x;
		pointVector.normal_y = cloudB_point.y - cloudA_point.y;
		pointVector.normal_z = cloudB_point.z - cloudA_point.z;

		int X = 0, Y = 0, Z = 0;
		X = (int)(cloudA_point.x / div1);
		Y = (int)(cloudA_point.y / div1);
		Z = (int)(cloudA_point.z / div1);

		cloud_smallgrid_vector[Z][Y][X]->push_back(pointVector);
	}

	//max search
	int max = 0;
	for (int k = 0; k < levs / div1; k++) {
		for (int j = 0; j < rows / div1; j++) {
			for (int i = 0; i < cols / div1; i++) {
				size_t N = cloud_smallgrid_vector[k][j][i]->size();
				if (max < N) {
					max = N;
				}
			}
		}
	}
	//printf("max : %d\n", max);

	//calculate centroid and vector_ave
	for (int k = 0; k < levs/div1; k++) {
		for (int j = 0; j < rows/div1; j++) {
			for (int i = 0; i < cols/div1; i++) {
				size_t N = cloud_smallgrid_vector[k][j][i]->size();
				float gx = 0, gy = 0, gz = 0, vx_ave = 0, vy_ave = 0, vz_ave = 0;

				for (size_t n = 0; n < N; n++) {
					gx += cloud_smallgrid_vector[k][j][i]->points[n].x;
					gy += cloud_smallgrid_vector[k][j][i]->points[n].y;
					gz += cloud_smallgrid_vector[k][j][i]->points[n].z;
					vx_ave += cloud_smallgrid_vector[k][j][i]->points[n].normal_x;
					vy_ave += cloud_smallgrid_vector[k][j][i]->points[n].normal_y;
					vz_ave += cloud_smallgrid_vector[k][j][i]->points[n].normal_z;
					//printf("cloud_smallgrid_vector[k][j][i]->points[n].normal_x : %f cloud_smallgrid_vector[k][j][i]->points[n].normal_y : %f cloud_smallgrid_vector[k][j][i]->points[n].normal_z : %f\n", cloud_smallgrid_vector[k][j][i]->points[n].normal_x, cloud_smallgrid_vector[k][j][i]->points[n].normal_y, cloud_smallgrid_vector[k][j][i]->points[n].normal_z);
				}
				PointXYZRGBNormal pointVector_ave;
				if (N != 0) {
					pointVector_ave.x = gx / N;
					pointVector_ave.y = gy / N;
					pointVector_ave.z = gz / N;
					pointVector_ave.r = cloud_smallgrid[k][j][i]->points[cloud_smallgrid[k][j][i]->size() - 1].r;
					pointVector_ave.g = pointVector_ave.r;
					pointVector_ave.b = pointVector_ave.r;
					pointVector_ave.normal_x = vx_ave / (max / 2.0);
					pointVector_ave.normal_y = vy_ave / (max / 2.0);
					pointVector_ave.normal_z = vz_ave / (max / 2.0);
				}
				else {
					pointVector_ave.x = i * div1 + (div1 - 1) / 2.0;
					pointVector_ave.y = j * div1 + (div1 - 1) / 2.0;
					pointVector_ave.z = k * div1 + (div1 - 1) / 2.0;
					pointVector_ave.r = cloud_smallgrid[k][j][i]->points[cloud_smallgrid[k][j][i]->size() - 1].r;
					pointVector_ave.g = pointVector_ave.r;
					pointVector_ave.b = pointVector_ave.r;
					pointVector_ave.normal_x = 0;
					pointVector_ave.normal_y = 0;
					pointVector_ave.normal_z = 0;
				}
				/*printf("N : %d\n", N);
				printf("vector_ave.x:%f  vector_ave.y:%f  vector_ave.z:%f\n", pointVector_ave.x, pointVector_ave.y, pointVector_ave.z);
				printf("vector_ave.normal_x:%f  vector_ave.normal_y:%f  vector_ave.normal_z:%f\n", pointVector_ave.normal_x, pointVector_ave.normal_y, pointVector_ave.normal_z);
				printf("vector_ave.r:%d  vector_ave.g:%d  vector_ave.b:%d\n\n", pointVector_ave.r, pointVector_ave.g, pointVector_ave.b);*/

				cloud_smallgrid_vector[k][j][i]->push_back(pointVector_ave);//input pointVector_ave into last number of cloud_smallgrid
				//printf("N : %d\n", N);
				//printf("cloud_smallgrid_vector[k][j][i]->points[N].normal_x : %f\n", cloud_smallgrid_vector[k][j][i]->points[N].normal_x);
			}
		}
	}
}

void splitCloudintoCloudSmallGrid2(PointCloud<PointXYZRGB>::Ptr cloud, vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>> &cloud_smallgrid, PointCloud<PointXYZRGB>::Ptr cloud1_keypoints, PointCloud<PointXYZRGB>::Ptr cloud2_keypoints, vector<Correspondence> &cloud_corrs, vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>> &cloud_smallgrid_vector)
{
	for (size_t i = 0; i < cloud->size(); i++) {
		PointXYZRGB p;
		p.x = cloud->points[i].x;
		p.y = cloud->points[i].y;
		p.z = cloud->points[i].z;
		p.r = cloud->points[i].r;
		p.g = cloud->points[i].g;
		p.b = cloud->points[i].b;

		int X = 0, Y = 0, Z = 0;
		X = (int)(p.x / div2);
		Y = (int)(p.y / div2);
		Z = (int)(p.z / div2);

		cloud_smallgrid[Z][Y][X]->push_back(p);
	}

	//calculate centroid and vector_ave
	for (int k = 0; k < levs / div2; k++) {
		for (int j = 0; j < rows / div2; j++) {
			for (int i = 0; i < cols / div2; i++) {
				size_t N = cloud_smallgrid[k][j][i]->size();
				float gx = 0, gy = 0, gz = 0, pixel_ave = 0;

				for (size_t n = 0; n < N; n++) {
					gx += cloud_smallgrid[k][j][i]->points[n].x;
					gy += cloud_smallgrid[k][j][i]->points[n].y;
					gz += cloud_smallgrid[k][j][i]->points[n].z;
					pixel_ave += (float)cloud_smallgrid[k][j][i]->points[n].r;
				}

				//cerr << "N:" << N << "  pixel_ave:" << pixel_ave << endl;

				PointXYZRGB p_ave;
				if (N != 0) {
					p_ave.x = gx / N;
					p_ave.y = gy / N;
					p_ave.z = gz / N;
					//p_ave.r = (int)(pixel_ave / pow(div2 / 2, 3));
					p_ave.r = (int)(pixel_ave / N);
					p_ave.g = p_ave.r;
					p_ave.b = p_ave.r;
				}
				else {
					p_ave.x = i * div2 + (div2 - 1) / 2.0;
					p_ave.y = j * div2 + (div2 - 1) / 2.0;
					p_ave.z = k * div2 + (div2 - 1) / 2.0;
					p_ave.r = 0;
					p_ave.g = 0;
					p_ave.b = 0;
				}

				//printf("cloud_smallgrid[%d][%d][%d].pix_ave:%d\n", k, j, i, p_ave.r);
				//cerr << "cloud_smallgrid[" << k << "][" << j << "][" << i << "].pix_ave:" << p_ave.r << endl;
				cloud_smallgrid[k][j][i]->push_back(p_ave);//input pointVector_ave into last number of cloud_smallgrid
			}
		}
	}


	for (size_t i = 0; i < cloud_corrs.size(); i++) {
		PointXYZRGB cloudA_point = cloud1_keypoints->at(cloud_corrs[i].index_match);
		PointXYZRGB cloudB_point = cloud2_keypoints->at(cloud_corrs[i].index_query);
		//cerr << "cloud_corrs : " << i << endl;

		PointXYZRGBNormal pointVector;
		pointVector.x = cloudA_point.x;
		pointVector.y = cloudA_point.y;
		pointVector.z = cloudA_point.z;
		pointVector.r = cloudA_point.r;
		pointVector.g = cloudA_point.g;
		pointVector.b = cloudA_point.b;
		pointVector.normal_x = cloudB_point.x - cloudA_point.x;
		pointVector.normal_y = cloudB_point.y - cloudA_point.y;
		pointVector.normal_z = cloudB_point.z - cloudA_point.z;

		int X = 0, Y = 0, Z = 0;
		X = (int)(cloudA_point.x / div2);
		Y = (int)(cloudA_point.y / div2);
		Z = (int)(cloudA_point.z / div2);

		cloud_smallgrid_vector[Z][Y][X]->push_back(pointVector);
	}

	//max search
	int max = 0;
	for (int k = 0; k < levs / div2; k++) {
		for (int j = 0; j < rows / div2; j++) {
			for (int i = 0; i < cols / div2; i++) {
				size_t N = cloud_smallgrid_vector[k][j][i]->size();
				if (max < N) {
					max = N;
				}
			}
		}
	}
	//printf("max : %d\n", max);

	//calculate centroid and vector_ave
	for (int k = 0; k < levs / div2; k++) {
		for (int j = 0; j < rows / div2; j++) {
			for (int i = 0; i < cols / div2; i++) {
				float vx_ave = 0, vy_ave = 0, vz_ave = 0;

				for (size_t n = 0; n < cloud_smallgrid_vector[k][j][i]->size(); n++) {
					vx_ave += cloud_smallgrid_vector[k][j][i]->points[n].normal_x;
					vy_ave += cloud_smallgrid_vector[k][j][i]->points[n].normal_y;
					vz_ave += cloud_smallgrid_vector[k][j][i]->points[n].normal_z;
					//printf("cloud_smallgrid_vector[k][j][i]->points[n].normal_x : %f cloud_smallgrid_vector[k][j][i]->points[n].normal_y : %f cloud_smallgrid_vector[k][j][i]->points[n].normal_z : %f\n", cloud_smallgrid_vector[k][j][i]->points[n].normal_x, cloud_smallgrid_vector[k][j][i]->points[n].normal_y, cloud_smallgrid_vector[k][j][i]->points[n].normal_z);
				}
				PointXYZRGBNormal pointVector_ave;
				int index_ave = cloud_smallgrid[k][j][i]->size() - 1;
				pointVector_ave.x = cloud_smallgrid[k][j][i]->points[index_ave].x;
				pointVector_ave.y = cloud_smallgrid[k][j][i]->points[index_ave].y;
				pointVector_ave.z = cloud_smallgrid[k][j][i]->points[index_ave].z;
				pointVector_ave.r = cloud_smallgrid[k][j][i]->points[index_ave].r;
				pointVector_ave.g = pointVector_ave.r;
				pointVector_ave.b = pointVector_ave.r;
				pointVector_ave.normal_x = vx_ave / (max / 2.0);
				pointVector_ave.normal_y = vy_ave / (max / 2.0);
				pointVector_ave.normal_z = vz_ave / (max / 2.0);

				/*printf("N : %d\n", N);
				printf("vector_ave.x:%f  vector_ave.y:%f  vector_ave.z:%f\n", pointVector_ave.x, pointVector_ave.y, pointVector_ave.z);
				printf("vector_ave.normal_x:%f  vector_ave.normal_y:%f  vector_ave.normal_z:%f\n", pointVector_ave.normal_x, pointVector_ave.normal_y, pointVector_ave.normal_z);
				printf("vector_ave.r:%d  vector_ave.g:%d  vector_ave.b:%d\n\n", pointVector_ave.r, pointVector_ave.g, pointVector_ave.b);*/

				cloud_smallgrid_vector[k][j][i]->push_back(pointVector_ave);//input pointVector_ave into last number of cloud_smallgrid
																			//printf("N : %d\n", N);
																			//printf("cloud_smallgrid_vector[k][j][i]->points[N].normal_x : %f\n", cloud_smallgrid_vector[k][j][i]->points[N].normal_x);
			}
		}
	}
}

void splitCloudintoCloudSmallGridFastDebug(PointCloud<PointXYZRGB>::Ptr cloud, vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>> &cloud_smallgrid)
{
	for (size_t i = 0; i < cloud->size(); i++) {
		PointXYZRGB p;
		p.x = cloud->points[i].x;
		p.y = cloud->points[i].y;
		p.z = cloud->points[i].z;
		p.r = cloud->points[i].r;
		p.g = cloud->points[i].g;
		p.b = cloud->points[i].b;

		int X = 0, Y = 0, Z = 0;
		X = (int)(p.x / div1);
		Y = (int)(p.y / div1);
		Z = (int)(p.z / div1);

		cloud_smallgrid[Z][Y][X]->push_back(p);
	}

	//calculate centroid and vector_ave
	for (int k = 0; k < levs / div1; k++) {
		for (int j = 0; j < rows / div1; j++) {
			for (int i = 0; i < cols / div1; i++) {
				size_t N = cloud_smallgrid[k][j][i]->size();
				float gx = 0, gy = 0, gz = 0, pixel_ave = 0;

				for (size_t n = 0; n < N; n++) {
					gx += cloud_smallgrid[k][j][i]->points[n].x;
					gy += cloud_smallgrid[k][j][i]->points[n].y;
					gz += cloud_smallgrid[k][j][i]->points[n].z;
					pixel_ave += (float)cloud_smallgrid[k][j][i]->points[n].r;
				}

				//cerr << "N:" << N << "  pixel_ave:" << pixel_ave << endl;

				PointXYZRGB p_ave;
				if (N != 0) {
					p_ave.x = gx / N;
					p_ave.y = gy / N;
					p_ave.z = gz / N;
					//p_ave.r = (int)(pixel_ave / pow(div1 / 2, 3));
					p_ave.r = (int)(pixel_ave / N);
					p_ave.g = p_ave.r;
					p_ave.b = p_ave.r;
				}
				else {
					p_ave.x = i * div1 + (div1 - 1) / 2.0;
					p_ave.y = j * div1 + (div1 - 1) / 2.0;
					p_ave.z = k * div1 + (div1 - 1) / 2.0;
					p_ave.r = 0;
					p_ave.g = 0;
					p_ave.b = 0;
				}

				//printf("cloud_smallgrid[%d][%d][%d].pix_ave:%d\n", k, j, i, p_ave.r);
				//cerr << "cloud_smallgrid[" << k << "][" << j << "][" << i << "].pix_ave:" << p_ave.r << endl;
				cloud_smallgrid[k][j][i]->push_back(p_ave);//input pointVector_ave into last number of cloud_smallgrid
			}
		}
	}
}

void clusteringBloodRegion(vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_smallgrid_onlypoints)
{
	float Thresh_Vector = 0.2;
	int Thresh_Pixel = 10;

	for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {
		for (int k = 0; k < levs / div1; k++) {
			float K = k*div1 + (div1 - 1) / 2.0;
			float x1 = (cols - 2 * delta_x)*(b1 - K) / (2 * (b1 - delta_z));
			float x2 = cols - (cols + 2 * delta_x)*(b2 - K) / (2 * (b2 - delta_z));
			float y1 = (rows + 2 * delta_y)*(b3 - K) / (2 * (b3 - delta_z));
			float y2 = rows - (rows - 2 * delta_y)*(b4 - K) / (2 * (b4 - delta_z));

			for (int j = 0; j < rows / div1; j++) {
				for (int i = 0; i < cols / div1; i++) {
					int index_ave = cloud_smallgrid_vector[f][k][j][i]->size() - 1;
					float vx = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_x;
					float vy = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_y;
					float vz = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_z;
					float v = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
					int pix = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].r;

					//cerr << "z:" << k << "  y:" << j << "  x:" << i << "  v_ave:" << v << "  pix_ave:" << pix << endl;

					PointXYZRGB p;
					p.x = i*div1 + (div1 - 1) / 2.0;
					p.y = j*div1 + (div1 - 1) / 2.0;
					p.z = k*div1 + (div1 - 1) / 2.0;
					p.r = 255;
					p.g = 0;
					p.b = 0;
					if ((v<Thresh_Vector) && (pix<Thresh_Pixel) && (p.x>x1&&p.x<x2) && (p.y>y1&&p.y<y2) && sqrt(pow(((cols / 2 - delta_x) - p.x), 2) + pow(((rows / 2 + delta_y) - p.y), 2) + pow(p.z, 2))<R) {
						cloud_smallgrid_onlypoints[f]->push_back(p);
					}
				}
			}
		}
	}
}

void clusteringBloodRegion2(vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>>> &cloud_smallgrid_onlypoints_blood, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_smallgrid_onlypoints)
{
	float Thresh_Vector = 0.2;
	int Thresh_Pixel = 30;

	for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {
		for (int k = 0; k < levs / div1; k++) {
			float K = k*div1 + (div1 - 1) / 2.0;
			float x1 = (cols + 2 * delta_x)*(b1 - K) / (2 * (b1 - delta_z));
			float x2 = cols - (cols - 2 * delta_x)*(b2 - K) / (2 * (b2 - delta_z));
			float y1 = (rows + 2 * delta_y)*(b3 - K) / (2 * (b3 - delta_z));
			float y2 = rows - (rows - 2 * delta_y)*(b4 - K) / (2 * (b4 - delta_z));

			for (int j = 0; j < rows / div1; j++) {
				for (int i = 0; i < cols / div1; i++) {
					int index_ave = cloud_smallgrid_vector[f][k][j][i]->size() - 1;
					//cerr << "index_ave : " << index_ave << endl;
					float vx = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_x;
					float vy = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_y;
					float vz = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_z;
					float v = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
					int pix = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].r;

					PointXYZRGB p;
					p.x = i*div1 + (div1 - 1) / 2.0;
					p.y = j*div1 + (div1 - 1) / 2.0;
					p.z = k*div1 + (div1 - 1) / 2.0;
					p.r = 0;
					p.g = 0;
					p.b = 255;
					if ((v<Thresh_Vector) && (pix<Thresh_Pixel) && (p.x>x1&&p.x<x2) && (p.y>y1&&p.y<y2) && sqrt(pow(((cols/2-delta_x)-p.x),2)+pow(((rows/2+delta_y)-p.y),2)+pow(p.z,2))<R) {
						cloud_smallgrid_onlypoints_blood[f][k][j][i]->push_back(p);
					}
					else {// hollow point
						PointXYZRGB p0;
						p0.x = 0;
						p0.y = 0;
						p0.z = 0;
						p0.r = 0;
						p0.g = 0;
						p0.b = 0;
						cloud_smallgrid_onlypoints_blood[f][k][j][i]->push_back(p0);
					}
				}
			}
		}
	}

	//bloodregion whenever
	for (int k = 0; k < levs / div1; k++) {
		for (int j = 0; j < rows / div1; j++) {
			for (int i = 0; i < cols / div1; i++) {
				int counter = 0;
				for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {
					if (cloud_smallgrid_onlypoints_blood[f][k][j][i]->points[0].b == 255) {
						counter++;
					}
				}
				if (counter >= cloud_smallgrid_vector.size() - 0) {
					for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {

						PointXYZRGB p;
						p.x = i*div1 + (div1 - 1) / 2.0;
						p.y = j*div1 + (div1 - 1) / 2.0;
						p.z = k*div1 + (div1 - 1) / 2.0;
						p.r = 255;
						p.g = 0;
						p.b = 0;
						cloud_smallgrid_onlypoints[f]->push_back(p);
					}
				}
				else {
					for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {
						if (cloud_smallgrid_onlypoints_blood[f][k][j][i]->points[0].b == 255) {
							// plot red points for other points
							//cloud_smallgrid_onlypoints[f]->push_back(cloud_smallgrid_onlypoints_blood[f][k][j][i]->points[0]);
						}
					}
				}
			}
		}
	}
	cloud_smallgrid_onlypoints_blood.clear();
}

void clusteringBloodRegion3(vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>>> &cloud_smallgrid_onlypoints_blood, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_smallgrid_onlypoints,int count)
{
	/*float Thresh_Vector = 0.2;
	int Thresh_Pixel = 30;*/
	//Goal Test 30
	float Thresh_Vector = 0.2;
	int Thresh_Pixel = 30;

	for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {
		for (int k = 0; k < levs / div1; k++) {
			float K = k*div1 + (div1 - 1) / 2.0;
			float x1 = (cols + 2 * delta_x)*(b1 - K) / (2 * (b1 - delta_z));
			float x2 = cols - (cols - 2 * delta_x)*(b2 - K) / (2 * (b2 - delta_z));
			float y1 = (rows + 2 * delta_y)*(b3 - K) / (2 * (b3 - delta_z));
			float y2 = rows - (rows - 2 * delta_y)*(b4 - K) / (2 * (b4 - delta_z));

			for (int j = 0; j < rows / div1; j++) {
				for (int i = 0; i < cols / div1; i++) {
					int index_ave = cloud_smallgrid_vector[f][k][j][i]->size() - 1;
					//cerr << "index_ave : " << index_ave << endl;
					float vx = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_x;
					float vy = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_y;
					float vz = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_z;
					float v = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
					int pix = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].r;

					PointXYZRGB p;
					p.x = i*div1 + (div1 - 1) / 2.0;
					p.y = j*div1 + (div1 - 1) / 2.0;
					p.z = k*div1 + (div1 - 1) / 2.0;
					p.r = 0;
					p.g = 0;
					p.b = 255;
					if ((v<Thresh_Vector) && (pix<Thresh_Pixel) && (p.x>x1&&p.x<x2) && (p.y>y1&&p.y<y2) && sqrt(pow(((cols / 2 - delta_x) - p.x), 2) + pow(((rows / 2 + delta_y) - p.y), 2) + pow(p.z, 2))<R) {
						cloud_smallgrid_onlypoints_blood[f][k][j][i]->push_back(p);
					}
					else {// hollow point
						PointXYZRGB p0;
						p0.x = 0;
						p0.y = 0;
						p0.z = 0;
						p0.r = 0;
						p0.g = 0;
						p0.b = 0;
						cloud_smallgrid_onlypoints_blood[f][k][j][i]->push_back(p0);
					}
				}
			}
		}
	}


	//bloodregion whenever
	for (int k = 0; k < levs / div1; k++) {
		for (int j = 0; j < rows / div1; j++) {
			for (int i = 0; i < cols / div1; i++) {
				int counter = 0;
				for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {
					if (cloud_smallgrid_onlypoints_blood[f][k][j][i]->points[0].b == 255) {
						counter++;
					}
				}
				if (counter >= count) {
					for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {

						PointXYZRGB p;
						p.x = i*div1 + (div1 - 1) / 2.0;
						p.y = j*div1 + (div1 - 1) / 2.0;
						p.z = k*div1 + (div1 - 1) / 2.0;
						p.r = 255;
						p.g = 0;
						p.b = 0;
						cloud_smallgrid_onlypoints[f]->push_back(p);
					}
				}
				else {
					for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {
						if (cloud_smallgrid_onlypoints_blood[f][k][j][i]->points[0].b == 255) {
							// plot red points for other points
							//cloud_smallgrid_onlypoints[f]->push_back(cloud_smallgrid_onlypoints_blood[f][k][j][i]->points[0]);
						}
					}
				}
			}
		}
	}
	cloud_smallgrid_onlypoints_blood.clear();
}

void clusteringBloodRegionFastDebug(vector<vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>>> &cloud_smallgrid, vector<vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>>> &cloud_smallgrid_onlypoints_blood, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_smallgrid_onlypoints, int count)
{
	/*float Thresh_Vector = 0.2;
	int Thresh_Pixel = 30;*/
	//Goal Test 30
	int Thresh_Pixel = 15;

	bool whenever_blood_region_(true);

	for (int f = 0; f < cloud_smallgrid.size(); f++) {
		for (int k = 0; k < levs / div1; k++) {
			float K = k*div1 + (div1 - 1) / 2.0;
			float x1 = (cols + 2 * delta_x)*(b1 - K) / (2 * (b1 - delta_z));
			float x2 = cols - (cols - 2 * delta_x)*(b2 - K) / (2 * (b2 - delta_z));
			float y1 = (rows + 2 * delta_y)*(b3 - K) / (2 * (b3 - delta_z));
			float y2 = rows - (rows - 2 * delta_y)*(b4 - K) / (2 * (b4 - delta_z));

			for (int j = 0; j < rows / div1; j++) {
				for (int i = 0; i < cols / div1; i++) {
					int index_ave = cloud_smallgrid[f][k][j][i]->size() - 1;
					//cerr << "index_ave : " << index_ave << endl;
					int pix = cloud_smallgrid[f][k][j][i]->points[index_ave].r;

					PointXYZRGB p;
					p.x = i*div1 + (div1 - 1) / 2.0;
					p.y = j*div1 + (div1 - 1) / 2.0;
					p.z = k*div1 + (div1 - 1) / 2.0;
					p.r = 255;
					p.g = 0;
					p.b = 0;
					if ((pix<Thresh_Pixel) && (p.x>x1&&p.x<x2) && (p.y>y1&&p.y<y2) && sqrt(pow(((cols / 2 - delta_x) - p.x), 2) + pow(((rows / 2 + delta_y) - p.y), 2) + pow(p.z, 2))<R) {
						cloud_smallgrid_onlypoints_blood[f][k][j][i]->push_back(p);
						if (!whenever_blood_region_)cloud_smallgrid_onlypoints[f]->push_back(p);
					}
					else {// hollow point
						PointXYZRGB p0;
						p0.x = 0;
						p0.y = 0;
						p0.z = 0;
						p0.r = 0;
						p0.g = 0;
						p0.b = 0;
						cloud_smallgrid_onlypoints_blood[f][k][j][i]->push_back(p0);
						if(!whenever_blood_region_)cloud_smallgrid_onlypoints[f]->push_back(p0);
					}
				}
			}
		}
	}

	if (whenever_blood_region_) {
		//bloodregion whenever
		for (int k = 0; k < levs / div1; k++) {
			for (int j = 0; j < rows / div1; j++) {
				for (int i = 0; i < cols / div1; i++) {
					int counter = 0;
					for (int f = 0; f < cloud_smallgrid.size(); f++) {
						if (cloud_smallgrid_onlypoints_blood[f][k][j][i]->points[0].r == 255) {
							counter++;
						}
					}
					if (counter >= count) {
						for (int f = 0; f < cloud_smallgrid.size(); f++) {

							PointXYZRGB p;
							p.x = i*div1 + (div1 - 1) / 2.0;
							p.y = j*div1 + (div1 - 1) / 2.0;
							p.z = k*div1 + (div1 - 1) / 2.0;
							p.r = 255;
							p.g = 0;
							p.b = 0;
							cloud_smallgrid_onlypoints[f]->push_back(p);
						}
					}
					else {
						for (int f = 0; f < cloud_smallgrid.size(); f++) {
							if (cloud_smallgrid_onlypoints_blood[f][k][j][i]->points[0].b == 255) {
								// plot red points for other points
								//cloud_smallgrid_onlypoints[f]->push_back(cloud_smallgrid_onlypoints_blood[f][k][j][i]->points[0]);
							}
						}
					}
				}
			}
		}
	}
	cloud_smallgrid_onlypoints_blood.clear();
}

void clusteringBloodRegion4(vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_smallgrid_onlypoints)
{
	float Thresh_Vector = 0.5;
	int Thresh_Pixel = 20;

	for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {
		for (int k = 0; k < levs / div1; k++) {
			float K = k*div1 + (div1 - 1) / 2.0;
			float x1 = (cols - 2 * delta_x)*(b1 - K) / (2 * (b1 - delta_z));
			float x2 = cols - (cols + 2 * delta_x)*(b2 - K) / (2 * (b2 - delta_z));
			float y1 = (rows + 2 * delta_y)*(b3 - K) / (2 * (b3 - delta_z));
			float y2 = rows - (rows - 2 * delta_y)*(b4 - K) / (2 * (b4 - delta_z));

			for (int j = 0; j < rows / div1; j++) {
				for (int i = 0; i < cols / div1; i++) {
					int index_ave = cloud_smallgrid_vector[f][k][j][i]->size() - 1;
					float vx = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_x;
					float vy = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_y;
					float vz = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_z;
					float v = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
					int pix = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].r;

					//cerr << "z:" << k << "  y:" << j << "  x:" << i << "  v_ave:" << v << "  pix_ave:" << pix << endl;
					int X, Y, Z;
					X = i*div1 + (div1 - 1) / 2.0;
					Y = j*div1 + (div1 - 1) / 2.0;
					Z = k*div1 + (div1 - 1) / 2.0;
					
					if ((v<Thresh_Vector) && (pix<Thresh_Pixel) && (X>x1&&X<x2) && (Y>y1&&Y<y2) && sqrt(pow(((cols / 2 - delta_x) - X), 2) + pow(((rows / 2 + delta_y) - Y), 2) + pow(Z, 2))<R) {
						for (int c = 0; c < div1; c++) {
							for (int b = 0; b < div1; b++) {
								for (int a = 0; a < div1; a++) {
									PointXYZRGB p;
									p.x = i*div1 + a;
									p.y = j*div1 + b;
									p.z = k*div1 + c;
									p.r = 255;
									p.g = 0;
									p.b = 0;
									cloud_smallgrid_onlypoints[f]->push_back(p);
								}
							}
						}
					}
				}
			}
		}
	}
}

void clusteringHighPixel(vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>>> &cloud_smallgrid_onlypoints_highpixel, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_smallgrid_onlypoints)
{
	float Thresh_Vector = 0.3;
	int Thresh_Pixel = 70;

	for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {
		for (int k = 0; k < levs / div2; k++) {
			for (int j = 0; j < rows / div2; j++) {
				for (int i = 0; i < cols / div2; i++) {
					int index_ave = cloud_smallgrid_vector[f][k][j][i]->size() - 1;
					float vx = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_x;
					float vy = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_y;
					float vz = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_z;
					float v = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
					int pix = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].r;

					PointXYZRGB p;
					p.x = i*div2 + (div2 - 1) / 2.0;
					p.y = j*div2 + (div2 - 1) / 2.0;
					p.z = k*div2 + (div2 - 1) / 2.0;
					p.r = 0;
					p.g = 255;
					p.b = 0;
					if ((v<Thresh_Vector) && (pix>Thresh_Pixel)) {
						cloud_smallgrid_onlypoints_highpixel[f][k][j][i]->push_back(p);
					}
					else {// hollow point
						PointXYZRGB p0;
						p0.x = 0;
						p0.y = 0;
						p0.z = 0;
						p0.r = 0;
						p0.g = 0;
						p0.b = 0;
						cloud_smallgrid_onlypoints_highpixel[f][k][j][i]->push_back(p0);
					}
				}
			}
		}
	}

	//bloodregion whenever
	for (int k = 0; k < levs / div2; k++) {
		for (int j = 0; j < rows / div2; j++) {
			for (int i = 0; i < cols / div2; i++) {
				int counter = 0;
				for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {
					if (cloud_smallgrid_onlypoints_highpixel[f][k][j][i]->points[0].g == 255) {
						counter++;
					}
				}
				if (counter >= cloud_smallgrid_vector.size() - 1) {
					for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {

						PointXYZRGB p;
						p.x = i*div2 + (div2 - 1) / 2.0;
						p.y = j*div2 + (div2 - 1) / 2.0;
						p.z = k*div2 + (div2 - 1) / 2.0;
						p.r = 255;
						p.g = 255;
						p.b = 255;
						cloud_smallgrid_onlypoints[f]->push_back(p);
					}
				}
				else {
					for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {
						if (cloud_smallgrid_onlypoints_highpixel[f][k][j][i]->points[0].g == 255) {
							// plot red points for other points
							//cloud_smallgrid_onlypoints[f]->push_back(cloud_smallgrid_onlypoints_highpixel[f][k][j][i]->points[0]);
						}
					}
				}
			}
		}
	}
	cloud_smallgrid_onlypoints_highpixel.clear();
}

void clusteringAtrialSeptum(vector<vector<int>> &timing, vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_smallgrid_onlypoints)
{
	float Thresh_Vector_S = 0.5, Thresh_Vector_M = 1.0, Thresh_Vector_L = 2.0;
	int Thresh_Pixel_M = 80, Thresh_Pixel_L = 130;
	//only cases of frame = 8
	for (int f = 0; f < timing.size(); f++) {
		for (int k = 0; k < levs / div2; k++) {
			for (int j = 0; j < rows / div2; j++) {
				for (int i = 0; i < cols / div2; i++) {
					int index_ave = cloud_smallgrid_vector[f][k][j][i]->size() - 1;
					float vx = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_x;
					float vy = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_y;
					float vz = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_z;
					float v = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
					int pix = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].r;

					PointXYZRGB p;
					p.x = i*div2 + (div2 - 1) / 2.0;
					p.y = j*div2 + (div2 - 1) / 2.0;
					p.z = k*div2 + (div2 - 1) / 2.0;
					p.r = 255;
					p.g = 255;
					p.b = 0;

					if (timing[f][0] == 0 && timing[f][1] == 0 && pix>Thresh_Pixel_M) {

						//cloud_smallgrid_onlypoints[f]->push_back(p);
					}
					if (timing[f][0] == 0 && timing[f][1] == 1 && v>Thresh_Vector_M && pix>Thresh_Pixel_M && vx<0 && vz<0) {

						cloud_smallgrid_onlypoints[f]->push_back(p);
					}
					if (timing[f][0] == 0 && timing[f][1] == 2 && v>Thresh_Vector_M && pix>Thresh_Pixel_M && vx<0 && vz<0) {
						cloud_smallgrid_onlypoints[f]->push_back(p);
					}
					if (timing[f][0] == 0 && timing[f][1] == 3 && v<Thresh_Vector_M && pix>Thresh_Pixel_L && vx<0 && vz<0) {
						cloud_smallgrid_onlypoints[f]->push_back(p);
					}
					if (timing[f][0] == 1 && timing[f][1] == 0 && v>Thresh_Vector_M && pix>Thresh_Pixel_L && vx>0 && vz>0) {
						cloud_smallgrid_onlypoints[f]->push_back(p);
					}
					if (timing[f][0] == 1 && timing[f][1] == 1 && v>Thresh_Vector_M && pix>Thresh_Pixel_L && vx>0 && vx / vz>1) {
						cloud_smallgrid_onlypoints[f]->push_back(p);
					}
					if (timing[f][0] == 1 && timing[f][1] == 2 && v<Thresh_Vector_M && pix>Thresh_Pixel_L && vx>0 && vz / vx<1 && vz / vx>-1) {
						cloud_smallgrid_onlypoints[f]->push_back(p);
					}
					if (timing[f][0] == 1 && timing[f][1] == 3 && v>Thresh_Vector_M && pix>Thresh_Pixel_L && vx>0 && vz>0) {
						cloud_smallgrid_onlypoints[f]->push_back(p);
					}
				}
			}
		}
	}
}

void clusteringVentricularSeptum(vector<vector<int>> &timing, vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_smallgrid_onlypoints)
{
	float Thresh_Vector_S = 0.5, Thresh_Vector_M = 1.0, Thresh_Vector_L = 2.0;
	int Thresh_Pixel_M = 80, Thresh_Pixel_L = 130;
	//only cases of frame = 8
	for (int f = 0; f < timing.size(); f++) {
		for (int k = 0; k < levs / div2; k++) {
			for (int j = 0; j < rows / div2; j++) {
				for (int i = 0; i < cols / div2; i++) {
					int index_ave = cloud_smallgrid_vector[f][k][j][i]->size() - 1;
					float vx = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_x;
					float vy = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_y;
					float vz = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].normal_z;
					float v = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
					int pix = cloud_smallgrid_vector[f][k][j][i]->points[index_ave].r;

					PointXYZRGB p;
					p.x = i*div2 + (div2 - 1) / 2.0;
					p.y = j*div2 + (div2 - 1) / 2.0;
					p.z = k*div2 + (div2 - 1) / 2.0;
					p.r = 0;
					p.g = 255;
					p.b = 255;

					if (p.x < cols / 2) {
						if (timing[f][0] == 0 && timing[f][1] == 0 && v > Thresh_Vector_L && pix > Thresh_Pixel_M && vx < 0 && vz>0) {
							cloud_smallgrid_onlypoints[f]->push_back(p);
						}
						if (timing[f][0] == 0 && timing[f][1] == 1 && v > Thresh_Vector_M && pix > Thresh_Pixel_L && vx<0 && vz / vx>-0.5 && vz / vx < 0.5) {
							cloud_smallgrid_onlypoints[f]->push_back(p);
						}
						if (timing[f][0] == 0 && timing[f][1] == 2 && v > Thresh_Vector_M && pix > Thresh_Pixel_L && vx < 0 && vz < 0) {
							cloud_smallgrid_onlypoints[f]->push_back(p);
						}
						if (timing[f][0] == 0 && timing[f][1] == 3 && v < Thresh_Vector_S && pix>Thresh_Pixel_M) {
							cloud_smallgrid_onlypoints[f]->push_back(p);
						}
						if (timing[f][0] == 1 && timing[f][1] == 0 && v > Thresh_Vector_L && pix > Thresh_Pixel_M && vx > 0 && vz < 0) {
							cloud_smallgrid_onlypoints[f]->push_back(p);
						}
						if (timing[f][0] == 1 && timing[f][1] == 1 && v > Thresh_Vector_M && pix > Thresh_Pixel_M && vx > 0 && vz < 0) {
							cloud_smallgrid_onlypoints[f]->push_back(p);
						}
						if (timing[f][0] == 1 && timing[f][1] == 2 && v > Thresh_Vector_M && pix>Thresh_Pixel_M && vx > 0 && vz / vx>-0.3 && vz / vx<0) {
							cloud_smallgrid_onlypoints[f]->push_back(p);
						}
						if (timing[f][0] == 1 && timing[f][1] == 3 && v < Thresh_Vector_M && pix < Thresh_Pixel_M && vx > 0 && vz > 0) {
							cloud_smallgrid_onlypoints[f]->push_back(p);
						}
					}
				}
			}
		}
	}
}

void clusteringLeftVentricle(vector<vector<int>> &timing, vector<PointCloud<PointXYZRGB>::Ptr> cloud_blood_clustered, vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_Ventricle)
{
	float Thresh_Vector_S = 0.5, Thresh_Vector_M = 1.0, Thresh_Vector_L = 2.0;
	int Thresh_Pixel_S = 30, Thresh_Pixel_M = 70, Thresh_Pixel_L = 130;

	// the Centroid of cloud_clustered_Ventricle
	int index_centroid_Ventricle = cloud_blood_clustered[0]->size() - 1;
	int index_centroid_Atrium = cloud_blood_clustered[1]->size() - 1;
	int index_centroid_Aorta = cloud_blood_clustered[2]->size() - 1;
	PointXYZ C_Ventricle, C_Atrium, C_Aorta;
	C_Ventricle.x = cloud_blood_clustered[0]->points[index_centroid_Ventricle].x;
	C_Ventricle.y = cloud_blood_clustered[0]->points[index_centroid_Ventricle].y;
	C_Ventricle.z = cloud_blood_clustered[0]->points[index_centroid_Ventricle].z;
	C_Atrium.x = cloud_blood_clustered[1]->points[index_centroid_Atrium].x;
	C_Atrium.y = cloud_blood_clustered[1]->points[index_centroid_Atrium].y;
	C_Atrium.z = cloud_blood_clustered[1]->points[index_centroid_Atrium].z;
	C_Aorta.x = cloud_blood_clustered[2]->points[index_centroid_Aorta].x;
	C_Aorta.y = cloud_blood_clustered[2]->points[index_centroid_Aorta].y;
	C_Aorta.z = cloud_blood_clustered[2]->points[index_centroid_Aorta].z;

	//radius search from
	float R_Ventricle = levs * 0.35;
	float R_Atrium = levs * 0.25;
	float R_Aorta = levs * 0.2;

	bool mode_succeed_(true);

	for (int F = 0; F < cloud_smallgrid_vector.size(); F++) {
		for (int Z = 0; Z < levs / div2; Z++) {
			for (int Y = 0; Y < rows / div2; Y++) {
				for (int X = 0; X < cols / div2; X++) {
					int index_ave = cloud_smallgrid_vector[F][Z][Y][X]->size() - 1;

					PointXYZRGBNormal p;
					p.x = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].x;
					p.y = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].y;
					p.z = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].z;
					p.r = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].r;
					p.g = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].g;
					p.b = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].b;
					p.normal_x = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].normal_x;
					p.normal_y = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].normal_y;
					p.normal_z = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].normal_z;

					/*printf("C.x : %f   C.y : %f   C.z : %f\n", C.x, C.y, C.z);
					printf("p.x : %f   p.y : %f   p.z : %f\n", p.x, p.y, p.z);*/

					float r_Ventricle = sqrt(pow(C_Ventricle.x - p.x, 2) + pow(C_Ventricle.y - p.y, 2) + pow(C_Ventricle.z - p.z, 2));
					float r_Atrium = sqrt(pow(C_Atrium.x - p.x, 2) + pow(C_Atrium.y - p.y, 2) + pow(C_Atrium.z - p.z, 2));
					float r_Aorta = sqrt(pow(C_Aorta.x - p.x, 2) + pow(C_Aorta.y - p.y, 2) + pow(C_Aorta.z - p.z, 2));

					/*printf("RADIUS = %f\n", RADIUS);
					printf("radius = %f\n\n", radius);*/

					float alpha = M_PI / 6.0;
					if (r_Ventricle < R_Ventricle && r_Atrium > R_Atrium && r_Aorta > R_Aorta) {
						float theta1 = atan((C_Ventricle.z - p.z) / (C_Ventricle.y - p.y)); if (theta1 < 0) theta1 += M_PI;
						float phi1 = atan(p.normal_z / p.normal_y); if (phi1 < 0) phi1 += M_PI;
						float theta2 = atan((C_Ventricle.z - p.z) / (C_Ventricle.x - p.x)); if (theta2 < 0) theta2 += M_PI;
						float phi2 = atan(p.normal_z / p.normal_x); if (phi2 < 0) phi2 += M_PI;
						bool PushBack_Ventricle_(true);

						if (Thresh_Pixel_S < p.r && p.r < Thresh_Pixel_M) {
							if (timing[F][0] == 0 && timing[F][1] == 0 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 0 && timing[F][1] == 1 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 0 && timing[F][1] == 2 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 0 && timing[F][1] == 3 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 0 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 1 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 2 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 3 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else PushBack_Ventricle_ = false;
						}
						else {
							PushBack_Ventricle_ = false;
						}

						if (PushBack_Ventricle_) {
							PointXYZRGB pointVentricle;
							pointVentricle.x = p.x;
							pointVentricle.y = p.y;
							pointVentricle.z = p.z;
							pointVentricle.r = 255;
							pointVentricle.g = 120;
							pointVentricle.b = 0;

							if (mode_succeed_) {
								for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {
									cloud_Ventricle[f]->push_back(pointVentricle);
								}
							}
							else {
								cloud_Ventricle[F]->push_back(pointVentricle);
							}

						}
					}
				}
			}
		}
		cerr << "cloud_Ventricle[" << F << "]->size() : " << cloud_Ventricle[F]->size() << endl;
	}
}

void clusteringLeftVentricle2(vector<vector<int>> &timing, vector<PointCloud<PointXYZRGB>::Ptr> cloud_blood_clustered, vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_Ventricle)
{
	float Thresh_Vector_S = 0.5, Thresh_Vector_M = 1.0, Thresh_Vector_L = 2.0;
	int Thresh_Pixel_S = 30, Thresh_Pixel_M = 70, Thresh_Pixel_L = 130;

	// the Centroid of cloud_clustered_Ventricle
	int index_centroid_Ventricle = cloud_blood_clustered[0]->size() - 1;
	int index_centroid_Atrium = cloud_blood_clustered[1]->size() - 1;
	int index_centroid_Aorta = cloud_blood_clustered[2]->size() - 1;
	PointXYZ C_Ventricle, C_Atrium, C_Aorta;
	C_Ventricle.x = cloud_blood_clustered[0]->points[index_centroid_Ventricle].x;
	C_Ventricle.y = cloud_blood_clustered[0]->points[index_centroid_Ventricle].y;
	C_Ventricle.z = cloud_blood_clustered[0]->points[index_centroid_Ventricle].z;
	C_Atrium.x = cloud_blood_clustered[1]->points[index_centroid_Atrium].x;
	C_Atrium.y = cloud_blood_clustered[1]->points[index_centroid_Atrium].y;
	C_Atrium.z = cloud_blood_clustered[1]->points[index_centroid_Atrium].z;
	C_Aorta.x = cloud_blood_clustered[2]->points[index_centroid_Aorta].x;
	C_Aorta.y = cloud_blood_clustered[2]->points[index_centroid_Aorta].y;
	C_Aorta.z = cloud_blood_clustered[2]->points[index_centroid_Aorta].z;

	//radius search from
	vector<int> radius_indices;
	vector<float> radius_sqr_dists;

	/*float R_Ventricle = levs * 0.35;
	float R_Atrium = levs * 0.25;
	float R_Aorta = levs * 0.2;*/

	float R_Ventricle = levs;
	float R_Atrium = 0;
	float R_Aorta = 0;

	bool mode_(false);

	for (int F = 0; F < cloud_smallgrid_vector.size(); F++) {
		for (int Z = 0; Z < levs / div2; Z++) {
			for (int Y = 0; Y < rows / div2; Y++) {
				for (int X = 0; X < cols / div2; X++) {
					int index_ave = cloud_smallgrid_vector[F][Z][Y][X]->size() - 1;
					
					PointXYZRGBNormal p;
					p.x = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].x;
					p.y = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].y;
					p.z = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].z;
					p.r = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].r;
					p.g = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].g;
					p.b = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].b;
					p.normal_x = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].normal_x;
					p.normal_y = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].normal_y;
					p.normal_z = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].normal_z;

					/*printf("C.x : %f   C.y : %f   C.z : %f\n", C.x, C.y, C.z);
					printf("p.x : %f   p.y : %f   p.z : %f\n", p.x, p.y, p.z);*/

					float r_Ventricle = sqrt(pow(C_Ventricle.x - p.x, 2) + pow(C_Ventricle.y - p.y, 2) + pow(C_Ventricle.z - p.z, 2));
					float r_Atrium = sqrt(pow(C_Atrium.x - p.x, 2) + pow(C_Atrium.y - p.y, 2) + pow(C_Atrium.z - p.z, 2));
					float r_Aorta = sqrt(pow(C_Aorta.x - p.x, 2) + pow(C_Aorta.y - p.y, 2) + pow(C_Aorta.z - p.z, 2));

					/*printf("RADIUS = %f\n", RADIUS);
					printf("radius = %f\n\n", radius);*/

					float alpha = M_PI / 6.0;
					if (r_Ventricle < R_Ventricle && r_Atrium > R_Atrium && r_Aorta > R_Aorta) {
						float theta1 = atan((C_Ventricle.z - p.z) / (C_Ventricle.y - p.y)); if (theta1 < 0) theta1 += M_PI;
						float phi1 = atan(p.normal_z / p.normal_y); if (phi1 < 0) phi1 += M_PI;
						float theta2 = atan((C_Ventricle.z - p.z) / (C_Ventricle.x - p.x)); if (theta2 < 0) theta2 += M_PI;
						float phi2 = atan(p.normal_z / p.normal_x); if (phi2 < 0) phi2 += M_PI;
						bool PushBack_Ventricle_(true);

						if (Thresh_Pixel_S < p.r && p.r < Thresh_Pixel_M) {
							if (timing[F][0] == 0 && timing[F][1] == 0 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 0 && timing[F][1] == 1 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 0 && timing[F][1] == 2 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 0 && timing[F][1] == 3 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 0 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 1 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 2 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 3 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else PushBack_Ventricle_ = false;
						}
						else {
							PushBack_Ventricle_ = false;
						}
						
						if (PushBack_Ventricle_) {
							PointXYZRGB pointVentricle;
							pointVentricle.r = 255;
							pointVentricle.g = 120;
							pointVentricle.b = 0;
							for (int k = 0; k < div2; k++) {
								for (int j = 0; j < div2; j++) {
									for (int i = 0; i < div2; i++) {
										pointVentricle.x = X*div2 + i;
										pointVentricle.y = Y*div2 + j;
										pointVentricle.z = Z*div2 + k;
										if (mode_) {
											cloud_Ventricle[F]->push_back(pointVentricle);
										}
										else {
											for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {
												cloud_Ventricle[f]->push_back(pointVentricle);
											}
										}
										
									}
								}
							}
						}
					}
				}
			}
		}
		cerr << "cloud_Ventricle[" << F << "]->size() : " << cloud_Ventricle[F]->size() << endl;
	}
}

void clusteringLeftAtrium(vector<vector<int>> &timing, vector<PointCloud<PointXYZRGB>::Ptr> cloud_blood_clustered, vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_Atrium)
{
	float Thresh_Vector_S = 0.5, Thresh_Vector_M = 1.0, Thresh_Vector_L = 2.0;
	int Thresh_Pixel_M = 80, Thresh_Pixel_L = 130;

	// the Centroid of cloud_clustered_Ventricle
	int index_centroid_Ventricle = cloud_blood_clustered[0]->size() - 1;
	int index_centroid_Atrium = cloud_blood_clustered[1]->size() - 1;
	int index_centroid_Aorta = cloud_blood_clustered[2]->size() - 1;
	PointXYZ C_Ventricle, C_Atrium, C_Aorta;
	C_Ventricle.x = cloud_blood_clustered[0]->points[index_centroid_Ventricle].x;
	C_Ventricle.y = cloud_blood_clustered[0]->points[index_centroid_Ventricle].y;
	C_Ventricle.z = cloud_blood_clustered[0]->points[index_centroid_Ventricle].z;
	C_Atrium.x = cloud_blood_clustered[1]->points[index_centroid_Atrium].x;
	C_Atrium.y = cloud_blood_clustered[1]->points[index_centroid_Atrium].y;
	C_Atrium.z = cloud_blood_clustered[1]->points[index_centroid_Atrium].z;
	C_Aorta.x = cloud_blood_clustered[2]->points[index_centroid_Aorta].x;
	C_Aorta.y = cloud_blood_clustered[2]->points[index_centroid_Aorta].y;
	C_Aorta.z = cloud_blood_clustered[2]->points[index_centroid_Aorta].z;

	//radius search from
	vector<int> radius_indices;
	vector<float> radius_sqr_dists;

	float R_Ventricle = levs * 0.35;
	float R_Atrium = levs * 0.25;
	float R_Aorta = levs * 0.2;

	bool mode_succeed_(true);

	for (int F = 0; F < cloud_smallgrid_vector.size(); F++) {
		for (int Z = 0; Z < levs / div2; Z++) {
			for (int Y = 0; Y < rows / div2; Y++) {
				for (int X = 0; X < cols / div2; X++) {
					int index_ave = cloud_smallgrid_vector[F][Z][Y][X]->size() - 1;

					PointXYZRGBNormal p;
					p.x = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].x;
					p.y = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].y;
					p.z = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].z;
					p.r = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].r;
					p.g = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].g;
					p.b = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].b;
					p.normal_x = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].normal_x;
					p.normal_y = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].normal_y;
					p.normal_z = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].normal_z;

					/*printf("C.x : %f   C.y : %f   C.z : %f\n", C.x, C.y, C.z);
					printf("p.x : %f   p.y : %f   p.z : %f\n", p.x, p.y, p.z);*/

					float r_Ventricle = sqrt(pow(C_Ventricle.x - p.x, 2) + pow(C_Ventricle.y - p.y, 2) + pow(C_Ventricle.z - p.z, 2));
					float r_Atrium = sqrt(pow(C_Atrium.x - p.x, 2) + pow(C_Atrium.y - p.y, 2) + pow(C_Atrium.z - p.z, 2));
					float r_Aorta = sqrt(pow(C_Aorta.x - p.x, 2) + pow(C_Aorta.y - p.y, 2) + pow(C_Aorta.z - p.z, 2));

					/*printf("RADIUS = %f\n", RADIUS);
					printf("radius = %f\n\n", radius);*/

					float alpha = M_PI / 6.0;
					if (r_Ventricle > R_Ventricle && r_Atrium < R_Atrium) {
						float theta1 = atan((C_Atrium.z - p.z) / (C_Atrium.y - p.y)); if (theta1 < 0) theta1 += M_PI;
						float phi1 = atan(p.normal_z / p.normal_y); if (phi1 < 0) phi1 += M_PI;
						float theta2 = atan((C_Atrium.z - p.z) / (C_Atrium.x - p.x)); if (theta2 < 0) theta2 += M_PI;
						float phi2 = atan(p.normal_z / p.normal_x); if (phi2 < 0) phi2 += M_PI;
						bool PushBack_Atrium_(true);

						if (Thresh_Pixel_M < p.r && p.r < Thresh_Pixel_L) {
							if (timing[F][0] == 0 && timing[F][1] == 0 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 0 && timing[F][1] == 1 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 0 && timing[F][1] == 2 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 0 && timing[F][1] == 3 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 0 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 1 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 2 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 3 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else PushBack_Atrium_ = false;
						}
						else {
							PushBack_Atrium_ = false;
						}

						if (PushBack_Atrium_) {
							PointXYZRGB pointAtrium;
							pointAtrium.x = p.x;
							pointAtrium.y = p.y;
							pointAtrium.z = p.z;
							pointAtrium.r = 0;
							pointAtrium.g = 120;
							pointAtrium.b = 255;
							if (mode_succeed_) {
								for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {
									cloud_Atrium[f]->push_back(pointAtrium);
								}
							}
							else {
								cloud_Atrium[F]->push_back(pointAtrium);
							}
						}
					}
				}
			}
		}
		cerr << "cloud_Atrium[" << F << "]->size() : " << cloud_Atrium[F]->size() << endl;
	}
}

void clusteringLeftAtrium2(vector<vector<int>> &timing, vector<PointCloud<PointXYZRGB>::Ptr> cloud_blood_clustered, vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_Atrium)
{
	float Thresh_Vector_S = 0.5, Thresh_Vector_M = 1.0, Thresh_Vector_L = 2.0;
	int Thresh_Pixel_S=20, Thresh_Pixel_M = 80, Thresh_Pixel_L = 130;

	// the Centroid of cloud_clustered_Ventricle
	int index_centroid_Ventricle = cloud_blood_clustered[0]->size() - 1;
	int index_centroid_Atrium = cloud_blood_clustered[1]->size() - 1;
	int index_centroid_Aorta = cloud_blood_clustered[2]->size() - 1;
	PointXYZ C_Ventricle, C_Atrium, C_Aorta;
	C_Ventricle.x = cloud_blood_clustered[0]->points[index_centroid_Ventricle].x;
	C_Ventricle.y = cloud_blood_clustered[0]->points[index_centroid_Ventricle].y;
	C_Ventricle.z = cloud_blood_clustered[0]->points[index_centroid_Ventricle].z;
	C_Atrium.x = cloud_blood_clustered[1]->points[index_centroid_Atrium].x;
	C_Atrium.y = cloud_blood_clustered[1]->points[index_centroid_Atrium].y;
	C_Atrium.z = cloud_blood_clustered[1]->points[index_centroid_Atrium].z;
	C_Aorta.x = cloud_blood_clustered[2]->points[index_centroid_Aorta].x;
	C_Aorta.y = cloud_blood_clustered[2]->points[index_centroid_Aorta].y;
	C_Aorta.z = cloud_blood_clustered[2]->points[index_centroid_Aorta].z;

	//radius search from
	vector<int> radius_indices;
	vector<float> radius_sqr_dists;

	float R_Ventricle = levs * 0.35;
	float R_Atrium = levs * 0.25;
	float R_Aorta = levs * 0.2;

	bool mode_(false);

	for (int F = 0; F < cloud_smallgrid_vector.size(); F++) {
		for (int Z = 0; Z < levs / div2; Z++) {
			for (int Y = 0; Y < rows / div2; Y++) {
				for (int X = 0; X < cols / div2; X++) {
					int index_ave = cloud_smallgrid_vector[F][Z][Y][X]->size() - 1;

					PointXYZRGBNormal p;
					p.x = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].x;
					p.y = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].y;
					p.z = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].z;
					p.r = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].r;
					p.g = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].g;
					p.b = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].b;
					p.normal_x = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].normal_x;
					p.normal_y = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].normal_y;
					p.normal_z = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].normal_z;

					/*printf("C.x : %f   C.y : %f   C.z : %f\n", C.x, C.y, C.z);
					printf("p.x : %f   p.y : %f   p.z : %f\n", p.x, p.y, p.z);*/

					float r_Ventricle = sqrt(pow(C_Ventricle.x - p.x, 2) + pow(C_Ventricle.y - p.y, 2) + pow(C_Ventricle.z - p.z, 2));
					float r_Atrium = sqrt(pow(C_Atrium.x - p.x, 2) + pow(C_Atrium.y - p.y, 2) + pow(C_Atrium.z - p.z, 2));
					float r_Aorta = sqrt(pow(C_Aorta.x - p.x, 2) + pow(C_Aorta.y - p.y, 2) + pow(C_Aorta.z - p.z, 2));

					/*printf("RADIUS = %f\n", RADIUS);
					printf("radius = %f\n\n", radius);*/

					float alpha = M_PI / 6.0;
					if (r_Ventricle > R_Ventricle && r_Atrium < R_Atrium) {
						float theta1 = atan((C_Atrium.z - p.z) / (C_Atrium.y - p.y)); if (theta1 < 0) theta1 += M_PI;
						float phi1 = atan(p.normal_z / p.normal_y); if (phi1 < 0) phi1 += M_PI;
						float theta2 = atan((C_Atrium.z - p.z) / (C_Atrium.x - p.x)); if (theta2 < 0) theta2 += M_PI;
						float phi2 = atan(p.normal_z / p.normal_x); if (phi2 < 0) phi2 += M_PI;
						bool PushBack_Atrium_(true);

						if (Thresh_Pixel_M < p.r && p.r < Thresh_Pixel_L) {
							if (timing[F][0] == 0 && timing[F][1] == 0 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 0 && timing[F][1] == 1 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 0 && timing[F][1] == 2 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 0 && timing[F][1] == 3 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 0 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 1 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 2 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 3 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else PushBack_Atrium_ = false;
						}
						else {
							PushBack_Atrium_ = false;
						}

						if (PushBack_Atrium_) {
							PointXYZRGB pointAtrium;
							pointAtrium.r = 0;
							pointAtrium.g = 120;
							pointAtrium.b = 255;

							for (int k = 0; k < div2; k++) {
								for (int j = 0; j < div2; j++) {
									for (int i = 0; i < div2; i++) {
										pointAtrium.x = X*div2 + i;
										pointAtrium.y = Y*div2 + j;
										pointAtrium.z = Z*div2 + k;
										if (mode_) {
											cloud_Atrium[F]->push_back(pointAtrium);
										}
										else {
											for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {
												cloud_Atrium[f]->push_back(pointAtrium);
											}
										}

									}
								}
							}
						}
					}
				}
			}
		}
		cerr << "cloud_Atrium[" << F << "]->size() : " << cloud_Atrium[F]->size() << endl;
	}
}

void clusteringAorta(vector<vector<int>> &timing, vector<PointCloud<PointXYZRGB>::Ptr> cloud_blood_clustered, vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_Aorta)
{
	float Thresh_Vector_S = 0.5, Thresh_Vector_M = 1.0, Thresh_Vector_L = 2.0;
	int Thresh_Pixel_S = 20, Thresh_Pixel_M = 80, Thresh_Pixel_L = 130;

	// the Centroid of cloud_clustered_Ventricle
	int index_centroid_Ventricle = cloud_blood_clustered[0]->size() - 1;
	int index_centroid_Atrium = cloud_blood_clustered[1]->size() - 1;
	int index_centroid_Aorta = cloud_blood_clustered[2]->size() - 1;
	PointXYZ C_Ventricle, C_Atrium, C_Aorta;
	C_Ventricle.x = cloud_blood_clustered[0]->points[index_centroid_Ventricle].x;
	C_Ventricle.y = cloud_blood_clustered[0]->points[index_centroid_Ventricle].y;
	C_Ventricle.z = cloud_blood_clustered[0]->points[index_centroid_Ventricle].z;
	C_Atrium.x = cloud_blood_clustered[1]->points[index_centroid_Atrium].x;
	C_Atrium.y = cloud_blood_clustered[1]->points[index_centroid_Atrium].y;
	C_Atrium.z = cloud_blood_clustered[1]->points[index_centroid_Atrium].z;
	C_Aorta.x = cloud_blood_clustered[2]->points[index_centroid_Aorta].x;
	C_Aorta.y = cloud_blood_clustered[2]->points[index_centroid_Aorta].y;
	C_Aorta.z = cloud_blood_clustered[2]->points[index_centroid_Aorta].z;


	float R_Ventricle = levs * 0.35;
	float R_Atrium = levs * 0.25;
	float R_Aorta = levs * 0.2;

	bool mode_succeed_(true);

	for (int F = 0; F < cloud_smallgrid_vector.size(); F++) {
		for (int Z = 0; Z < levs / div2; Z++) {
			for (int Y = 0; Y < rows / div2; Y++) {
				for (int X = 0; X < cols / div2; X++) {
					int index_ave = cloud_smallgrid_vector[F][Z][Y][X]->size() - 1;

					PointXYZRGBNormal p;
					p.x = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].x;
					p.y = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].y;
					p.z = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].z;
					p.r = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].r;
					p.g = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].g;
					p.b = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].b;
					p.normal_x = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].normal_x;
					p.normal_y = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].normal_y;
					p.normal_z = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].normal_z;

					/*printf("C.x : %f   C.y : %f   C.z : %f\n", C.x, C.y, C.z);
					printf("p.x : %f   p.y : %f   p.z : %f\n", p.x, p.y, p.z);*/

					float r_Ventricle = sqrt(pow(C_Ventricle.x - p.x, 2) + pow(C_Ventricle.y - p.y, 2) + pow(C_Ventricle.z - p.z, 2));
					float r_Atrium = sqrt(pow(C_Atrium.x - p.x, 2) + pow(C_Atrium.y - p.y, 2) + pow(C_Atrium.z - p.z, 2));
					float r_Aorta = sqrt(pow(C_Aorta.x - p.x, 2) + pow(C_Aorta.y - p.y, 2) + pow(C_Aorta.z - p.z, 2));

					/*printf("RADIUS = %f\n", RADIUS);
					printf("radius = %f\n\n", radius);*/

					float alpha = M_PI / 6.0;
					if (r_Ventricle > R_Ventricle && r_Atrium > R_Atrium && r_Aorta < R_Aorta) {
						float theta1 = atan((C_Atrium.z - p.z) / (C_Atrium.y - p.y)); if (theta1 < 0) theta1 += M_PI;
						float phi1 = atan(p.normal_z / p.normal_y); if (phi1 < 0) phi1 += M_PI;
						float theta2 = atan((C_Atrium.z - p.z) / (C_Atrium.x - p.x)); if (theta2 < 0) theta2 += M_PI;
						float phi2 = atan(p.normal_z / p.normal_x); if (phi2 < 0) phi2 += M_PI;
						bool PushBack_Aorta_(true);

						if (Thresh_Pixel_M < p.r && p.r < Thresh_Pixel_L) {
							if (timing[F][0] == 0 && timing[F][1] == 0 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 0 && timing[F][1] == 1 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 0 && timing[F][1] == 2 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 0 && timing[F][1] == 3 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 0 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 1 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 2 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 3 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else PushBack_Aorta_ = false;
						}
						else {
							PushBack_Aorta_ = false;
						}

						if (PushBack_Aorta_) {
							PointXYZRGB pointAorta;
							pointAorta.x = p.x;
							pointAorta.y = p.y;
							pointAorta.z = p.z;
							pointAorta.r = 70;
							pointAorta.g = 255;
							pointAorta.b = 70;

							if (mode_succeed_) {
								for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {
									cloud_Aorta[f]->push_back(pointAorta);
								}
							}
							else {
								cloud_Aorta[F]->push_back(pointAorta);
							}
						}
					}
				}
			}
		}
		cerr << "cloud_Aorta[" << F << "]->size() : " << cloud_Aorta[F]->size() << endl;
	}
}

void clusteringAorta2(vector<vector<int>> &timing, vector<PointCloud<PointXYZRGB>::Ptr> cloud_blood_clustered, vector<vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>>> &cloud_smallgrid_vector, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_Aorta)
{
	float Thresh_Vector_S = 0.5, Thresh_Vector_M = 1.0, Thresh_Vector_L = 2.0;
	int Thresh_Pixel_S = 20, Thresh_Pixel_M = 80, Thresh_Pixel_L = 130;

	// the Centroid of cloud_clustered_Ventricle
	int index_centroid_Ventricle = cloud_blood_clustered[0]->size() - 1;
	int index_centroid_Atrium = cloud_blood_clustered[1]->size() - 1;
	int index_centroid_Aorta = cloud_blood_clustered[2]->size() - 1;
	PointXYZ C_Ventricle, C_Atrium, C_Aorta;
	C_Ventricle.x = cloud_blood_clustered[0]->points[index_centroid_Ventricle].x;
	C_Ventricle.y = cloud_blood_clustered[0]->points[index_centroid_Ventricle].y;
	C_Ventricle.z = cloud_blood_clustered[0]->points[index_centroid_Ventricle].z;
	C_Atrium.x = cloud_blood_clustered[1]->points[index_centroid_Atrium].x;
	C_Atrium.y = cloud_blood_clustered[1]->points[index_centroid_Atrium].y;
	C_Atrium.z = cloud_blood_clustered[1]->points[index_centroid_Atrium].z;
	C_Aorta.x = cloud_blood_clustered[2]->points[index_centroid_Aorta].x;
	C_Aorta.y = cloud_blood_clustered[2]->points[index_centroid_Aorta].y;
	C_Aorta.z = cloud_blood_clustered[2]->points[index_centroid_Aorta].z;


	float R_Ventricle = levs * 0.35;
	float R_Atrium = levs * 0.25;
	float R_Aorta = levs * 0.2;

	bool mode_(false);

	for (int F = 0; F < cloud_smallgrid_vector.size(); F++) {
		for (int Z = 0; Z < levs / div2; Z++) {
			for (int Y = 0; Y < rows / div2; Y++) {
				for (int X = 0; X < cols / div2; X++) {
					int index_ave = cloud_smallgrid_vector[F][Z][Y][X]->size() - 1;

					PointXYZRGBNormal p;
					p.x = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].x;
					p.y = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].y;
					p.z = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].z;
					p.r = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].r;
					p.g = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].g;
					p.b = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].b;
					p.normal_x = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].normal_x;
					p.normal_y = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].normal_y;
					p.normal_z = cloud_smallgrid_vector[F][Z][Y][X]->points[index_ave].normal_z;

					/*printf("C.x : %f   C.y : %f   C.z : %f\n", C.x, C.y, C.z);
					printf("p.x : %f   p.y : %f   p.z : %f\n", p.x, p.y, p.z);*/

					float r_Ventricle = sqrt(pow(C_Ventricle.x - p.x, 2) + pow(C_Ventricle.y - p.y, 2) + pow(C_Ventricle.z - p.z, 2));
					float r_Atrium = sqrt(pow(C_Atrium.x - p.x, 2) + pow(C_Atrium.y - p.y, 2) + pow(C_Atrium.z - p.z, 2));
					float r_Aorta = sqrt(pow(C_Aorta.x - p.x, 2) + pow(C_Aorta.y - p.y, 2) + pow(C_Aorta.z - p.z, 2));

					/*printf("RADIUS = %f\n", RADIUS);
					printf("radius = %f\n\n", radius);*/

					float alpha = M_PI / 6.0;
					if (r_Ventricle > R_Ventricle && r_Atrium > R_Atrium && r_Aorta < R_Aorta) {
						float theta1 = atan((C_Atrium.z - p.z) / (C_Atrium.y - p.y)); if (theta1 < 0) theta1 += M_PI;
						float phi1 = atan(p.normal_z / p.normal_y); if (phi1 < 0) phi1 += M_PI;
						float theta2 = atan((C_Atrium.z - p.z) / (C_Atrium.x - p.x)); if (theta2 < 0) theta2 += M_PI;
						float phi2 = atan(p.normal_z / p.normal_x); if (phi2 < 0) phi2 += M_PI;
						bool PushBack_Aorta_(true);

						if (Thresh_Pixel_M < p.r && p.r < Thresh_Pixel_L) {
							if (timing[F][0] == 0 && timing[F][1] == 0 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 0 && timing[F][1] == 1 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 0 && timing[F][1] == 2 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 0 && timing[F][1] == 3 && ((theta1 - alpha) < phi1 && phi1 < (theta1 + alpha)) && ((theta2 - alpha) < phi2 && phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 0 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 1 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 2 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else if (timing[F][0] == 1 && timing[F][1] == 3 && ((theta1 - alpha) < M_PI - phi1 && M_PI - phi1 < (theta1 + alpha)) && ((theta2 - alpha) < M_PI - phi2 && M_PI - phi2 < (theta2 + alpha)));
							else PushBack_Aorta_ = false;
						}
						else {
							PushBack_Aorta_ = false;
						}

						if (PushBack_Aorta_) {
							PointXYZRGB pointAorta;
							pointAorta.r = 70;
							pointAorta.g = 255;
							pointAorta.b = 70;
							for (int k = 0; k < div2; k++) {
								for (int j = 0; j < div2; j++) {
									for (int i = 0; i < div2; i++) {
										pointAorta.x = X*div2 + i;
										pointAorta.y = Y*div2 + j;
										pointAorta.z = Z*div2 + k;
										if (mode_) {
											cloud_Aorta[F]->push_back(pointAorta);
										}
										else {
											for (int f = 0; f < cloud_smallgrid_vector.size(); f++) {
												cloud_Aorta[f]->push_back(pointAorta);
											}
										}

									}
								}
							}
						}
					}
				}
			}
		}
		cerr << "cloud_Aorta[" << F << "]->size() : " << cloud_Aorta[F]->size() << endl;
	}
}

void displayGridInfo(visualization::PCLVisualizer *viewer, vector<vector<vector<PointCloud<PointXYZRGBNormal>::Ptr>>> &cloud_smallgrid_vector, int k, int j, int i)
{
	int index_ave = cloud_smallgrid_vector[k][j][i]->size() - 1;
	float Cx = cloud_smallgrid_vector[k][j][i]->points[index_ave].x;
	float Cy = cloud_smallgrid_vector[k][j][i]->points[index_ave].y;
	float Cz = cloud_smallgrid_vector[k][j][i]->points[index_ave].z;
	float vx_ave = cloud_smallgrid_vector[k][j][i]->points[index_ave].normal_x;
	float vy_ave = cloud_smallgrid_vector[k][j][i]->points[index_ave].normal_y;
	float vz_ave = cloud_smallgrid_vector[k][j][i]->points[index_ave].normal_z;
	float v_ave = sqrt(pow(vx_ave, 2) + pow(vy_ave, 2) + pow(vz_ave, 2));
	int pix_ave = cloud_smallgrid_vector[k][j][i]->points[index_ave].r;

	printf("--------------------------------------------------\n");
	printf("      x=%f   y=%f   z=%f\n", Cx / PixelSpacingX, Cy / PixelSpacingY, Cz / PixelSpacingZ);
	printf("     vx=%f  vy=%f  vz=%f\n", vx_ave, vy_ave, vz_ave);
	printf("    pix=%d\n", pix_ave);
	printf("--------------------------------------------------\n");

	PointCloud<PointXYZRGBA>::Ptr cloud_info(new PointCloud<PointXYZRGBA>);
	for (int Z = 0; Z < div1; Z++) {
		for (int Y = 0; Y < div1; Y++) {
			for (int X = 0; X < div1; X++) {
				PointXYZRGBA p;
				p.x = (i*div1 + X)*PixelSpacingX;
				p.y = (j*div1 + Y)*PixelSpacingY;
				p.z = (k*div1 + Z)*PixelSpacingZ;
				p.r = 255;
				p.g = 255;
				p.b = 255;
				p.a = 50;
				cloud_info->push_back(p);
			}
		}
	}
	viewer->addPointCloud(cloud_info, "cloud_info");
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud_info");

	PointXYZ p0, p1;
	p0.x = (i*div1 + (div1 - 1) / 2.0)*PixelSpacingX;
	p0.y = (j*div1 + (div1 - 1) / 2.0)*PixelSpacingY;
	p0.z = (k*div1 + (div1 - 1) / 2.0)*PixelSpacingZ;
	int T = 200;
	p1.x = p0.x + vx_ave*PixelSpacingX*T;
	p1.y = p0.y + vy_ave*PixelSpacingY*T;
	p1.z = p0.z + vz_ave*PixelSpacingZ*T;
	if ((p1.x - p0.x) >= 0 && (p1.y - p0.y) >= 0 && (p1.z - p0.z) >= 0) viewer->addLine<PointXYZ, PointXYZ>(p0, p1, 1.0, 0, 0, "cloud_info_vector");
	else if ((p1.x - p0.x) >= 0 && (p1.y - p0.y) < 0 && (p1.z - p0.z) >= 0) viewer->addLine<PointXYZ, PointXYZ>(p0, p1, 1.0, 1.0, 0, "cloud_info_vector");
	else if ((p1.x - p0.x) < 0 && (p1.y - p0.y) >= 0 && (p1.z - p0.z) >= 0) viewer->addLine<PointXYZ, PointXYZ>(p0, p1, 0.5, 1.0, 1.0, "cloud_info_vector");
	else if ((p1.x - p0.x) < 0 && (p1.y - p0.y) < 0 && (p1.z - p0.z) >= 0) viewer->addLine<PointXYZ, PointXYZ>(p0, p1, 0.5, 0.5, 1.0, "cloud_info_vector");
	else if ((p1.x - p0.x) >= 0 && (p1.y - p0.y) >= 0 && (p1.z - p0.z) < 0) viewer->addLine<PointXYZ, PointXYZ>(p0, p1, 1.0, 0.5, 0.5, "cloud_info_vector");
	else if ((p1.x - p0.x) >= 0 && (p1.y - p0.y) < 0 && (p1.z - p0.z) < 0) viewer->addLine<PointXYZ, PointXYZ>(p0, p1, 1.0, 1.0, 0.5, "cloud_info_vector");
	else if ((p1.x - p0.x) < 0 && (p1.y - p0.y) >= 0 && (p1.z - p0.z) < 0) viewer->addLine<PointXYZ, PointXYZ>(p0, p1, 0, 1.0, 1.0, "cloud_info_vector");
	else if ((p1.x - p0.x) < 0 && (p1.y - p0.y) < 0 && (p1.z - p0.z) < 0) viewer->addLine<PointXYZ, PointXYZ>(p0, p1, 0, 0, 1.0, "cloud_info_vector");
	else;
}

void euclideanClustering(PointCloud<PointXYZRGB>::Ptr cloud, float Tolerance, int size_min, int size_max, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_clustered)
{
	
	// Creating the KdTree object for the search method of the extraction
	search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>);
	tree->setInputCloud(cloud);
	

	vector<PointIndices> cluster_indices;
	EuclideanClusterExtraction<PointXYZRGB> ec;
	ec.setClusterTolerance(Tolerance); // 2cm
	ec.setMinClusterSize(size_min);
	ec.setMaxClusterSize(size_max);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	
	
	int i = 0;
	for (vector<PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		//initialize
		PointCloud<PointXYZRGB>::Ptr cloud_cluster(new PointCloud<PointXYZRGB>);
		cloud_clustered.push_back(cloud_cluster);

		for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
			cloud_clustered[i]->points.push_back(cloud->points[*pit]);
		}
		//cloud_clustered[i]->width = cloud_clustered[i]->points.size();
		//cloud_clustered[i]->height = 1;
		//cloud_clustered[i]->is_dense = true;

		//numbering clustered cloud from maximum size to minimum size
		//cout << "PointCloud representing the Cluster: " << cloud_clustered[i]->points.size() << " data points." << endl;
		
		i++;
	}

	for (size_t i = 0; i < cloud_clustered.size(); i++) {
		PointXYZRGB C;
		C.x = 0; C.y = 0; C.z = 0; C.r = 0; C.g = 0; C.b = 0;
		for (size_t j = 0; j < cloud_clustered[i]->size(); j++) {
			C.x += cloud_clustered[i]->points[j].x;
			C.y += cloud_clustered[i]->points[j].y;
			C.z += cloud_clustered[i]->points[j].z;
		}
		C.x /= cloud_clustered[i]->size();
		C.y /= cloud_clustered[i]->size();
		C.z /= cloud_clustered[i]->size();
		cloud_clustered[i]->push_back(C);
	}

	//set different color
	//printf("cloud_clustered.size() : %d\n", cloud_clustered.size());
	for (size_t l = 0; l < cloud_clustered.size(); l++) {
		for (size_t m = 0; m < cloud_clustered[l]->size(); m++) {
			switch (l) {
			case 0:cloud_clustered[l]->points[m].r = 255; cloud_clustered[l]->points[m].g = 0; cloud_clustered[l]->points[m].b = 0; break;
			case 1:cloud_clustered[l]->points[m].r = 255; cloud_clustered[l]->points[m].g = 255; cloud_clustered[l]->points[m].b = 0; break;
			case 2:cloud_clustered[l]->points[m].r = 0; cloud_clustered[l]->points[m].g = 255; cloud_clustered[l]->points[m].b = 255; break;
			case 3:cloud_clustered[l]->points[m].r = 0; cloud_clustered[l]->points[m].g = 0; cloud_clustered[l]->points[m].b = 255; break;
			case 4:cloud_clustered[l]->points[m].r = 255; cloud_clustered[l]->points[m].g = 0; cloud_clustered[l]->points[m].b = 255; break;
			case 5:cloud_clustered[l]->points[m].r = 0; cloud_clustered[l]->points[m].g = 255; cloud_clustered[l]->points[m].b = 0; break;
			default:cloud_clustered[l]->points[m].r = 120; cloud_clustered[l]->points[m].g = 120; cloud_clustered[l]->points[m].b = 120; break;
			}
		}
		cloud_clustered[l]->points[cloud_clustered[l]->size() - 1].r = 255; cloud_clustered[l]->points[cloud_clustered[l]->size() - 1].g = 255; cloud_clustered[l]->points[cloud_clustered[l]->size() - 1].b = 255;
	}
}

void recognitionBloodRegion(vector<PointCloud<PointXYZRGB>::Ptr> &cloud_clustered_before, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_clustered_after)
{
	vector<int> indices;
	indices.resize(cloud_clustered_before.size());

	int n = 3;
	for (size_t i = 0; i < cloud_clustered_before.size(); i++) {
		int index_centroid = cloud_clustered_before[i]->size() - 1;
		//LV
		if (cloud_clustered_before[i]->points[index_centroid].x<cols / 3 && cloud_clustered_before[i]->points[index_centroid].z>levs / 3.) {
			indices[0] = i;
		}
		//LA
		else if (cloud_clustered_before[i]->points[index_centroid].x > cols / 2 && cloud_clustered_before[i]->points[index_centroid].y > rows / 3 && cloud_clustered_before[i]->points[index_centroid].z > levs * 2 / 2.5) {
			indices[1] = i;
		}
		//Aorta
		else if (cloud_clustered_before[i]->points[index_centroid].x > cols / 2 && cloud_clustered_before[i]->points[index_centroid].y > rows / 3&& cloud_clustered_before[i]->points[index_centroid].y < rows*2 / 3 && cloud_clustered_before[i]->points[index_centroid].z > levs / 3 && cloud_clustered_before[i]->points[index_centroid].z < levs * 2 / 3) {
			indices[2] = i;
		}
		else {
			indices[n] = i;
			n++;
		}
	}

	for (size_t i = 0; i < cloud_clustered_before.size(); i++) {

		PointCloud<PointXYZRGB>::Ptr temp(new PointCloud<PointXYZRGB>);
		for (size_t j = 0; j < cloud_clustered_before[indices[i]]->size(); j++) {
			PointXYZRGB p;
			p.x = cloud_clustered_before[indices[i]]->points[j].x;
			p.y = cloud_clustered_before[indices[i]]->points[j].y;
			p.z = cloud_clustered_before[indices[i]]->points[j].z;
			p.r = cloud_clustered_before[indices[i]]->points[j].r;
			p.g = cloud_clustered_before[indices[i]]->points[j].g;
			p.b = cloud_clustered_before[indices[i]]->points[j].b;
			temp->push_back(p);
		}
		cloud_clustered_after.push_back(temp);
	}
}

void recognitionBloodRegion2(vector<PointCloud<PointXYZRGB>::Ptr> &cloud_clustered_before, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_clustered_after)
{
	vector<int> indices;
	indices.resize(cloud_clustered_before.size());
	vector<int> indices_usedInBefore;

	int NumberOfPoint = 20;
	int NumberOfAorta = 10;

	//LV, LA
	int min_index_LV;
	int min_distance_LV = cols;
	int max_index_LA;
	int max_z_LA = 0;
	int number_Aorta = 0;
	vector<float> distance_plane_v;
	vector<int> distance_plane_align;
	for (size_t i = 0; i < cloud_clustered_before.size(); i++) {
		int index_centroid_minsearch = cloud_clustered_before[i]->size() - 1;
		if (index_centroid_minsearch > NumberOfPoint) {
			//LV
			float distance_left = cloud_clustered_before[i]->points[index_centroid_minsearch].x - ((cols + 2 * delta_x)*(b1 - cloud_clustered_before[i]->points[index_centroid_minsearch].z) / (2 * (b1 - delta_z)));
			if (min_distance_LV > distance_left && cloud_clustered_before[i]->points[index_centroid_minsearch].z > levs / 2.8 && cloud_clustered_before[i]->points[index_centroid_minsearch].z < levs / 1.8) {
				min_distance_LV = distance_left;
				min_index_LV = i;
				//printf("i:%d  distance_left:%f\n", i, distance_left);
			}

			//LA
			if (max_z_LA < (cloud_clustered_before[i]->points[index_centroid_minsearch].z)) {
				max_z_LA = cloud_clustered_before[i]->points[index_centroid_minsearch].z;
				max_index_LA = i;
			}
		}
	}
	indices[0] = min_index_LV;
	indices[1] = max_index_LA;
	indices_usedInBefore.push_back(min_index_LV);
	indices_usedInBefore.push_back(max_index_LA);

	//Aorta1, Aorta2(maybe)
	PointXYZ p_Center, p_LV, p_LA;
	p_Center.x = cols / 2 + delta_x;
	p_Center.y = rows / 2 + delta_y;
	p_Center.z = 0;
	int index_LV = cloud_clustered_before[min_index_LV]->size() - 1;
	p_LV.x = cloud_clustered_before[min_index_LV]->points[index_LV].x;
	p_LV.y = cloud_clustered_before[min_index_LV]->points[index_LV].y;
	p_LV.z = cloud_clustered_before[min_index_LV]->points[index_LV].z;
	int index_LA = cloud_clustered_before[max_index_LA]->size() - 1;
	p_LA.x = cloud_clustered_before[max_index_LA]->points[index_LA].x;
	p_LA.y = cloud_clustered_before[max_index_LA]->points[index_LA].y;
	p_LA.z = cloud_clustered_before[max_index_LA]->points[index_LA].z;
	float a = -(p_Center.y*(p_LV.z - p_LA.z) + p_LV.y*(p_LA.z - p_Center.z) + p_LA.y*(p_Center.z - p_LV.z)) /
		(p_Center.x*(p_LV.y*p_LA.z - p_LV.z*p_LA.y) + p_LV.x*(p_LA.y*p_Center.z - p_LA.z*p_Center.y) + p_LA.x*(p_Center.y*p_LV.z - p_Center.z*p_LV.y));
	float b= (p_Center.x*(p_LV.z - p_LA.z) + p_LV.x*(p_LA.z - p_Center.z) + p_LA.x*(p_Center.z - p_LV.z)) /
		(p_Center.x*(p_LV.y*p_LA.z - p_LV.z*p_LA.y) + p_LV.x*(p_LA.y*p_Center.z - p_LA.z*p_Center.y) + p_LA.x*(p_Center.y*p_LV.z - p_Center.z*p_LV.y));
	float c = -(p_Center.x*(p_LV.y - p_LA.y) + p_LV.x*(p_LA.y - p_Center.y) + p_LA.x*(p_Center.y - p_LV.y)) /
		(p_Center.x*(p_LV.y*p_LA.z - p_LV.z*p_LA.y) + p_LV.x*(p_LA.y*p_Center.z - p_LA.z*p_Center.y) + p_LA.x*(p_Center.y*p_LV.z - p_Center.z*p_LV.y));
	float d = 1.0;
	//printf("p_Center  x:%f  y:%f  z:%f\n", p_Center.x, p_Center.y, p_Center.z);
	//printf("p_LV      x:%f  y:%f  z:%f\n",p_LV.x, p_LV.y, p_LV.z);
	//printf("p_LA      x:%f  y:%f  z:%f\n", p_LA.x, p_LA.y, p_LA.z);
	//float distance_Center = a*p_Center.x + b*p_Center.y + c*p_Center.z + d;
	//float distance_LV = a*p_LV.x + b*p_LV.y + c*p_LV.z + d;
	//float distance_LA = a*p_LA.x + b*p_LA.y + c*p_LA.z + d;
	//printf("distance_Center:%f  distance_LV:%f  distance_LA:%f\n", distance_Center, distance_LV, distance_LA);

	//for (size_t i = 0; i < cloud_clustered_before.size(); i++) {
	//	int index_centroid_minsearch = cloud_clustered_before[i]->size() - 1;
	//	//if (index_centroid_minsearch <= NumberOfPoint) {
	//		float distance_plane = a*cloud_clustered_before[i]->points[index_centroid_minsearch].x + b*cloud_clustered_before[i]->points[index_centroid_minsearch].y + c*cloud_clustered_before[i]->points[index_centroid_minsearch].z + d;
	//		//printf("distance_plane:%f\n", distance_plane);
	//		if (fabs(distance_plane) < 0.2 && cloud_clustered_before[i]->points[index_centroid_minsearch].z>levs/3 && cloud_clustered_before[i]->points[index_centroid_minsearch].z<levs*2 / 3.3) {
	//			printf("distance_plane:%f\n", distance_plane);
	//			if (!(i == min_index_LV || i == max_index_LA)) {
	//				indices[2 + index_from_Aorta] = i;
	//				index_from_Aorta++;
	//				indices_usedInBefore.push_back(i);
	//				//printf("i:%d\n", i);
	//			}
	//		}
	//	//}
	//}

	for (size_t i = 0; i < cloud_clustered_before.size(); i++) {
		int counter = 0;

		int index_centroid_minsearch = cloud_clustered_before[i]->size() - 1;
		float distance_plane;
		if (index_centroid_minsearch >= NumberOfAorta) {
			distance_plane = a*cloud_clustered_before[i]->points[index_centroid_minsearch].x + b*cloud_clustered_before[i]->points[index_centroid_minsearch].y + c*cloud_clustered_before[i]->points[index_centroid_minsearch].z + d;
			//printf("distance_plane:%f\n", distance_plane);
			if (cloud_clustered_before[i]->points[index_centroid_minsearch].z > levs / 3 && cloud_clustered_before[i]->points[index_centroid_minsearch].z < levs * 2 / 3.3) {
				printf("distance_plane:%f\n", distance_plane);
				if (!(i == min_index_LV || i == max_index_LA)) {
					//indices_usedInBefore.push_back(i);
					//printf("i:%d\n", i);
					counter++;
				}
			}
		}
		distance_plane_align.push_back(i);
		if (counter > 0) {
			distance_plane_v.push_back(distance_plane);
		}
		else {
			distance_plane_v.push_back(1000);
		}
	}
	for (size_t i = 0; i < distance_plane_v.size(); i++) {
		for (size_t j = 0; j < distance_plane_v.size() - 1; j++) {
			if (fabs(distance_plane_v[j]) > fabs(distance_plane_v[j + 1])) {
				float temp = distance_plane_v[j];
				distance_plane_v[j] = distance_plane_v[j + 1];
				distance_plane_v[j + 1] = temp;
				int index_temp = distance_plane_align[j];
				distance_plane_align[j] = distance_plane_align[j + 1];
				distance_plane_align[j + 1] = index_temp;
			}
		}
	}
	if (fabs(distance_plane_v[1]) - fabs(distance_plane_v[0]) < 0.1) {
		indices[2] = distance_plane_align[0];
		indices[3] = distance_plane_align[1];
		indices_usedInBefore.push_back(distance_plane_align[0]);
		indices_usedInBefore.push_back(distance_plane_align[1]);
		number_Aorta = 2;
	}
	else {
		indices[2] = distance_plane_align[0];
		indices_usedInBefore.push_back(distance_plane_align[0]);
		number_Aorta = 1;
	}

	//others
	for (size_t i = 0; i < cloud_clustered_before.size(); i++) {
		int counter = 0;
		for (size_t n = 0; n < indices_usedInBefore.size(); n++) {
			if (i == indices_usedInBefore[n]) {
				counter++;
			}
		}
		if (counter == 0) {
			indices[2 + number_Aorta] = i;
			number_Aorta++;
		}
	}

	//change before into after
	for (size_t i = 0; i < indices_usedInBefore.size(); i++) {

		PointCloud<PointXYZRGB>::Ptr temp(new PointCloud<PointXYZRGB>);
		for (size_t j = 0; j < cloud_clustered_before[indices[i]]->size(); j++) {
			PointXYZRGB p;
			p.x = cloud_clustered_before[indices[i]]->points[j].x;
			p.y = cloud_clustered_before[indices[i]]->points[j].y;
			p.z = cloud_clustered_before[indices[i]]->points[j].z;
			p.r = cloud_clustered_before[indices[i]]->points[j].r;
			p.g = cloud_clustered_before[indices[i]]->points[j].g;
			p.b = cloud_clustered_before[indices[i]]->points[j].b;
			temp->push_back(p);
		}
		cloud_clustered_after.push_back(temp);
	}
}


//void smoothing(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGBNormal>::Ptr cloud_filtered)
//{
//	// Output has the PointNormal type in order to store the normals calculated by MLS
//	// Create a KD-Tree
//	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
//
//	// Init object (second point type is for the normals, even if unused)
//	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;
//
//	// Set parameters
//	mls.setInputCloud(cloud);
//	mls.setPolynomialFit(true);
//	mls.setSearchMethod(tree);
//	mls.setSearchRadius(5.0);
//
//	// Reconstruct
//	mls.process(cloud_filtered);
//}

void bilateralFiltering(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr cloud_filtered)
{
	PointCloud<PointXYZI>::Ptr cloud_intensity(new PointCloud<PointXYZI>);
	PointCloud<PointXYZI>::Ptr cloud_filtered_intensity(new PointCloud<PointXYZI>);
	cloud_intensity->resize(cloud->size());
	cloud_filtered_intensity->resize(cloud->size());

	for (size_t n = 0; n < cloud->size(); n++) {
		cloud_intensity->points[n].x = cloud->points[n].x;
		cloud_intensity->points[n].y = cloud->points[n].y;
		cloud_intensity->points[n].z = cloud->points[n].z;
		cloud_intensity->points[n].intensity = cloud->points[n].r / 255.0;
		//printf("intensity : %f\n", cloud_intensity->points[n].intensity);
	}

	// Create the filtering object
	pcl::BilateralFilter<pcl::PointXYZI> bFilter;
	bFilter.setInputCloud(cloud_intensity);
	bFilter.setHalfSize(2.0);
	bFilter.setStdDev(5.0);//sigma
	bFilter.filter(*cloud_filtered_intensity);

	cloud_filtered->resize(cloud->size());
	for (size_t n = 0; n < cloud->size(); n++) {
		cloud_filtered->points[n].x = cloud_filtered_intensity->points[n].x;
		cloud_filtered->points[n].y = cloud_filtered_intensity->points[n].y;
		cloud_filtered->points[n].z = cloud_filtered_intensity->points[n].z;
		cloud_filtered->points[n].r = (int)(cloud_filtered_intensity->points[n].intensity * 255);
		cloud_filtered->points[n].g = cloud_filtered->points[n].r;
		cloud_filtered->points[n].b = cloud_filtered->points[n].r;
	}
}

void outerProductFromNormal(PointCloud<PointNormal>::Ptr cloud_PointNormal, int X,int Y,int Z, PointCloud<PointXYZRGB>::Ptr &cloud_info, PointCloud<Normal>::Ptr &cloud_Normal_info, PointCloud<PointNormal>::Ptr &cloud_PointNormal_ave_info)
{
	cloud_info->clear();
	cloud_Normal_info->clear();
	cloud_PointNormal_ave_info->clear();

	cloud_info->resize(cloud_PointNormal->size());
	cloud_Normal_info->resize(cloud_PointNormal->size());

	PointNormal p;
	p.x = X;
	p.y = Y;
	p.z = Z;
	p.normal_x = 0;
	p.normal_y = 0;
	p.normal_z = 0;

	//radius search from
	KdTreeFLANN<PointNormal> radius_search;
	float radius = 0.3f;
	vector<int> radius_indices;
	vector<float> radius_sqr_dists;

	radius_search.setInputCloud(cloud_PointNormal);

	int found_radiusSerched_neighs = radius_search.radiusSearch(p, radius, radius_indices, radius_sqr_dists);
	cerr << "found_radiusSerched_neighs : " << found_radiusSerched_neighs << endl;

	PointNormal normal_ave;
	float outerProduct_x = 0, outerProduct_y = 0, outerProduct_z = 0, outerProduct_scalar = 0;
	if (found_radiusSerched_neighs) {
		for (int i = 0; i < found_radiusSerched_neighs; i++) {
			normal_ave.x += cloud_PointNormal->at(radius_indices[i]).x;
			normal_ave.y += cloud_PointNormal->at(radius_indices[i]).y;
			normal_ave.z += cloud_PointNormal->at(radius_indices[i]).z;
			normal_ave.normal_x += cloud_PointNormal->at(radius_indices[i]).normal_x;
			normal_ave.normal_y += cloud_PointNormal->at(radius_indices[i]).normal_y;
			normal_ave.normal_z += cloud_PointNormal->at(radius_indices[i]).normal_z;

			cloud_info->points[radius_indices[i]].x = cloud_PointNormal->at(radius_indices[i]).x;
			cloud_info->points[radius_indices[i]].y = cloud_PointNormal->at(radius_indices[i]).y;
			cloud_info->points[radius_indices[i]].z = cloud_PointNormal->at(radius_indices[i]).z;
			cloud_info->points[radius_indices[i]].r = 255;
			cloud_info->points[radius_indices[i]].g = 255;
			cloud_info->points[radius_indices[i]].b = 255;
			cloud_Normal_info->points[radius_indices[i]].normal_x = cloud_PointNormal->at(radius_indices[i]).normal_x;
			cloud_Normal_info->points[radius_indices[i]].normal_y = cloud_PointNormal->at(radius_indices[i]).normal_y;
			cloud_Normal_info->points[radius_indices[i]].normal_z = cloud_PointNormal->at(radius_indices[i]).normal_z;
		}
		normal_ave.x /= (float)found_radiusSerched_neighs;
		normal_ave.y /= (float)found_radiusSerched_neighs;
		normal_ave.z /= (float)found_radiusSerched_neighs;
		normal_ave.normal_x /= (float)found_radiusSerched_neighs;
		normal_ave.normal_y /= (float)found_radiusSerched_neighs;
		normal_ave.normal_z /= (float)found_radiusSerched_neighs;
		cloud_PointNormal_ave_info->push_back(normal_ave);

		for (int i = 0; i < found_radiusSerched_neighs; i++) {
			outerProduct_x += (normal_ave.normal_y*cloud_PointNormal->at(radius_indices[i]).normal_z - normal_ave.normal_z*cloud_PointNormal->at(radius_indices[i]).normal_y);
			outerProduct_y += (normal_ave.normal_z*cloud_PointNormal->at(radius_indices[i]).normal_x - normal_ave.normal_x*cloud_PointNormal->at(radius_indices[i]).normal_z);
			outerProduct_z += (normal_ave.normal_x*cloud_PointNormal->at(radius_indices[i]).normal_y - normal_ave.normal_y*cloud_PointNormal->at(radius_indices[i]).normal_x);
		}
		outerProduct_x /= (float)found_radiusSerched_neighs;
		outerProduct_y /= (float)found_radiusSerched_neighs;
		outerProduct_z /= (float)found_radiusSerched_neighs;
	}
	outerProduct_scalar = sqrt(pow(outerProduct_x, 2) + pow(outerProduct_y, 2) + pow(outerProduct_z, 2));
	printf("outerProduct_x:%f  outerProduct_y:%f  outerProduct_z:%f\n", outerProduct_x, outerProduct_y, outerProduct_z);
	printf("outerProduct_scalar:%f\n", outerProduct_scalar);
}

void locationEstimation_valve(vector<vector<int>> &timing, vector<PointCloud<PointXYZRGB>::Ptr> cloud_blood_clustered, vector<PointCloud<PointXYZRGB>::Ptr> cloud_Ventricle, vector<PointCloud<PointXYZRGB>::Ptr> cloud_Atrium, vector<PointCloud<PointXYZRGB>::Ptr> cloud_Aorta, vector<PointCloud<PointXYZRGB>::Ptr> cloud, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_MitralValve, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_AorticValve)
{
	// the Centroid of cloud_clustered_Ventricle
	int index_centroid_Ventricle = cloud_blood_clustered[0]->size() - 1;
	int index_centroid_Atrium = cloud_blood_clustered[1]->size() - 1;
	int index_centroid_Aorta = cloud_blood_clustered[2]->size() - 1;
	PointXYZ C_Ventricle, C_Atrium, C_Aorta;
	C_Ventricle.x = cloud_blood_clustered[0]->points[index_centroid_Ventricle].x;
	C_Ventricle.y = cloud_blood_clustered[0]->points[index_centroid_Ventricle].y;
	C_Ventricle.z = cloud_blood_clustered[0]->points[index_centroid_Ventricle].z;
	C_Atrium.x = cloud_blood_clustered[1]->points[index_centroid_Atrium].x;
	C_Atrium.y = cloud_blood_clustered[1]->points[index_centroid_Atrium].y;
	C_Atrium.z = cloud_blood_clustered[1]->points[index_centroid_Atrium].z;
	C_Aorta.x = cloud_blood_clustered[2]->points[index_centroid_Aorta].x;
	C_Aorta.y = cloud_blood_clustered[2]->points[index_centroid_Aorta].y;
	C_Aorta.z = cloud_blood_clustered[2]->points[index_centroid_Aorta].z;

	//set cuboid size
	float l = levs*0.4f;
	PointXYZRGB C_MitralValve, C_AorticValve;
	C_MitralValve.x = (C_Ventricle.x + C_Atrium.x) / 2.0;
	C_MitralValve.y = (C_Ventricle.y + C_Atrium.y) / 2.0;
	C_MitralValve.z = (C_Ventricle.z + C_Atrium.z) / 2.0;
	C_MitralValve.r = 255; C_MitralValve.g = 0; C_MitralValve.b = 0;
	C_AorticValve.x = C_Aorta.x;
	C_AorticValve.y = C_Aorta.y;
	C_AorticValve.z = C_Aorta.z;
	C_AorticValve.r = 0; C_AorticValve.g = 255; C_AorticValve.b = 0;
	printf("C_MitralValve     x : %f  y : %f  z : %f\n", C_MitralValve.x, C_MitralValve.y, C_MitralValve.z);
	printf("C_AorticValve     x : %f  y : %f  z : %f\n", C_AorticValve.x, C_AorticValve.y, C_AorticValve.z);



	//set cuboid of mitralValve
	for (size_t f = 0; f < cloud.size(); f++) {
		if (timing[f][0] == 1 && timing[f][1] == 2) {
			for (size_t n = 0; n < cloud[f]->size(); n++) {
				if ((C_MitralValve.x - l / 2.0 < cloud[f]->points[n].x&&cloud[f]->points[n].x < C_MitralValve.x + l / 2.0) && (C_MitralValve.y - l / 2.0 < cloud[f]->points[n].y&&cloud[f]->points[n].y < C_MitralValve.y + l / 2.0) && (C_MitralValve.z - l / 2.0 < cloud[f]->points[n].z&&cloud[f]->points[n].z < C_MitralValve.z + l / 2.0)) {
					PointXYZRGB p;
					p.x = cloud[f]->points[n].x;
					p.y = cloud[f]->points[n].y;
					p.z = cloud[f]->points[n].z;
					p.r = cloud[f]->points[n].r;
					p.g = cloud[f]->points[n].g;
					p.b = cloud[f]->points[n].b;
					for (size_t F = 0; F < cloud.size(); F++) {
						cloud_MitralValve[F]->push_back(p);
					}
				}
			}
			
		}
		cloud_MitralValve[f]->push_back(C_MitralValve);
		cerr << "cloud_MitralValve[" << f << "]->size() : " << cloud_MitralValve[f]->size() << endl;
	}
	//set cuboid of aorticValve
	for (size_t f = 0; f < cloud.size(); f++) {
		if (timing[f][0] == 1 && timing[f][1] == 2) {
			for (size_t n = 0; n < cloud[f]->size(); n++) {
				if ((C_AorticValve.x - l / 2.0 < cloud[f]->points[n].x&&cloud[f]->points[n].x < C_AorticValve.x + l / 2.0) && (C_AorticValve.y - l / 2.0 < cloud[f]->points[n].y&&cloud[f]->points[n].y < C_AorticValve.y + l / 2.0) && (C_AorticValve.z - l / 2.0 < cloud[f]->points[n].z&&cloud[f]->points[n].z < C_AorticValve.z + l / 2.0)) {
					PointXYZRGB p;
					p.x = cloud[f]->points[n].x;
					p.y = cloud[f]->points[n].y;
					p.z = cloud[f]->points[n].z;
					p.r = cloud[f]->points[n].r;
					p.g = cloud[f]->points[n].g;
					p.b = cloud[f]->points[n].b;
					cloud_AorticValve[f]->push_back(p);
					for (size_t F = 0; F < cloud.size(); F++) {
						cloud_AorticValve[F]->push_back(p);
					}
				}
			}
			
		}
		cloud_AorticValve[f]->push_back(C_AorticValve);
		cerr << "cloud_AorticValve[" << f << "]->size() : " << cloud_AorticValve[f]->size() << endl;
	}

	////radius search
	//KdTreeFLANN<PointXYZRGB> radius_search;
	//float radius_Ventricle = levs * 0.35/2.0;
	//float radius_Atrium = levs * 0.25/2.0;
	//float radius_Aorta = levs * 0.2/2.0;
	//vector<int> radius_indices;
	//vector<float> radius_sqr_dists;
	//PointCloud<PointXYZRGB>::Ptr cloud_search_Ventricle(new PointCloud<PointXYZRGB>);
	//for (int X = -(int)radius_Ventricle; X < +(int)radius_Ventricle; X++) {
	//	for (int Y = -(int)radius_Ventricle; Y < +(int)radius_Ventricle; Y++) {
	//		for (int Z = -(int)radius_Ventricle; Z < +(int)radius_Ventricle; Z++) {
	//			PointXYZRGB p;
	//			p.x = C_Ventricle.x + X;
	//			p.y = C_Ventricle.y + Y;
	//			p.z = C_Ventricle.z + Z;
	//			p.r = 255;
	//			p.g = 255;
	//			p.b = 255;
	//			cloud_search_Ventricle->push_back(p);
	//		}
	//	}
	//}
	//PointCloud<PointXYZRGB>::Ptr cloud_search_Atrium(new PointCloud<PointXYZRGB>);
	//for (int X = -(int)radius_Atrium; X < +(int)radius_Atrium; X++) {
	//	for (int Y = -(int)radius_Atrium; Y < +(int)radius_Atrium; Y++) {
	//		for (int Z = -(int)radius_Atrium; Z < +(int)radius_Atrium; Z++) {
	//			PointXYZRGB p;
	//			p.x = C_Atrium.x + X;
	//			p.y = C_Atrium.y + Y;
	//			p.z = C_Atrium.z + Z;
//			p.r = 255;
//			p.g = 255;
//			p.b = 255;
//			cloud_search_Atrium->push_back(p);
//		}
//	}
//}
//PointCloud<PointXYZRGB>::Ptr cloud_search_Aorta(new PointCloud<PointXYZRGB>);
//for (int X = -(int)radius_Aorta; X < +(int)radius_Aorta; X++) {
//	for (int Y = -(int)radius_Aorta; Y < +(int)radius_Aorta; Y++) {
//		for (int Z = -(int)radius_Aorta; Z < +(int)radius_Aorta; Z++) {
//			PointXYZRGB p;
//			p.x = C_Aorta.x + X;
//			p.y = C_Aorta.y + Y;
//			p.z = C_Aorta.z + Z;
//			p.r = 255;
//			p.g = 255;
//			p.b = 255;
//			cloud_search_Aorta->push_back(p);
//		}
//	}
//}

////Ventricle
//for (size_t f = 0; f < cloud_Ventricle.size(); f++) {
//	int out_ventricle = cloud_Ventricle[f]->size()*0.2;
//	PointXYZRGB c_possible, p_Ventricle;
//	c_possible.r = 255; c_possible.g = 255; c_possible.b = 255;
//	p_Ventricle.r = 255; p_Ventricle.g = 255; p_Ventricle.b = 255;
//	for (size_t i = 0; i < cloud_search_Ventricle->size(); i++) {
//		c_possible.x = cloud_search_Ventricle->points[i].x;
//		c_possible.y = cloud_search_Ventricle->points[i].y;
//		c_possible.z = cloud_search_Ventricle->points[i].z;
//		for (size_t j = 0; j < cloud_Ventricle[f]->size(); j++) {
//			p_Ventricle.x = cloud_Ventricle[f]->points[j].x;
//			p_Ventricle.y = cloud_Ventricle[f]->points[j].y;
//			p_Ventricle.z = cloud_Ventricle[f]->points[j].z;
//			float radius = sqrt(pow(()) + pow() + pow());
//		}
//	}

//	if (found_radiusSerched_neighs) {
//		PointXYZRGB p;
//		for (int n = 0; n < found_radiusSerched_neighs; n++) {
//			p.x = cloud_Ventricle[f]->at(radius_indices[n]).x;
//			p.y = cloud_Ventricle[f]->at(radius_indices[n]).y;
//			p.z = cloud_Ventricle[f]->at(radius_indices[n]).z;

//			float radius=sqrt(pow(())+pow()+pow())
//		}
//	}
//}
//
////Atrium


////Aorta


}

void locationEstimation_valve2(vector<vector<int>> &timing, vector<PointCloud<PointXYZRGB>::Ptr> cloud_blood_clustered, vector<PointCloud<PointXYZRGB>::Ptr> cloud, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_MitralValve, vector<PointCloud<PointXYZRGB>::Ptr> &cloud_AorticValve)
{
	// the Centroid of cloud_clustered_Ventricle
	int index_centroid_Ventricle = cloud_blood_clustered[0]->size() - 1;
	int index_centroid_Atrium = cloud_blood_clustered[1]->size() - 1;
	int index_centroid_Aorta1, index_centroid_Aorta2;
	index_centroid_Aorta1 = cloud_blood_clustered[2]->size() - 1;
	PointXYZ C_Ventricle, C_Atrium, C_Aorta1, C_Aorta2;
	C_Ventricle.x = cloud_blood_clustered[0]->points[index_centroid_Ventricle].x;
	C_Ventricle.y = cloud_blood_clustered[0]->points[index_centroid_Ventricle].y;
	C_Ventricle.z = cloud_blood_clustered[0]->points[index_centroid_Ventricle].z;
	C_Atrium.x = cloud_blood_clustered[1]->points[index_centroid_Atrium].x;
	C_Atrium.y = cloud_blood_clustered[1]->points[index_centroid_Atrium].y;
	C_Atrium.z = cloud_blood_clustered[1]->points[index_centroid_Atrium].z;
	C_Aorta1.x = cloud_blood_clustered[2]->points[index_centroid_Aorta1].x;
	C_Aorta1.y = cloud_blood_clustered[2]->points[index_centroid_Aorta1].y;
	C_Aorta1.z = cloud_blood_clustered[2]->points[index_centroid_Aorta1].z;
	//Aorta2 presence or not
	if (cloud_blood_clustered.size() == 4) {
		index_centroid_Aorta2 = cloud_blood_clustered[3]->size() - 1;
		C_Aorta2.x = cloud_blood_clustered[3]->points[index_centroid_Aorta2].x;
		C_Aorta2.y = cloud_blood_clustered[3]->points[index_centroid_Aorta2].y;
		C_Aorta2.z = cloud_blood_clustered[3]->points[index_centroid_Aorta2].z;
	}
	else if (cloud_blood_clustered.size() == 3)
	{
		index_centroid_Aorta2 = 0;
		C_Aorta2.x = 0;
		C_Aorta2.y = 0;
		C_Aorta2.z = 0;
	}

	//set cuboid size
	float l = levs*0.4f;
	PointXYZRGB C_MitralValve, C_AorticValve;
	C_MitralValve.r = 255; C_MitralValve.g = 0; C_MitralValve.b = 0;
	C_MitralValve.x = (C_Ventricle.x + C_Atrium.x) / 2.0;
	C_MitralValve.y = (C_Ventricle.y + C_Atrium.y) / 2.0;
	C_MitralValve.z = (C_Ventricle.z + C_Atrium.z) / 2.0;
	C_AorticValve.r = 0; C_AorticValve.g = 255; C_AorticValve.b = 0;
	if (cloud_blood_clustered.size() == 4) {
		C_AorticValve.x = (C_Aorta1.x + C_Aorta2.x) / 2.0;
		C_AorticValve.y = (C_Aorta1.y + C_Aorta2.y) / 2.0;
		C_AorticValve.z = (C_Aorta1.z + C_Aorta2.z) / 2.0;
	}
	else if (cloud_blood_clustered.size() == 3) {
		float l_LVtoLA = sqrt(pow((C_Ventricle.x - C_Atrium.x), 2) + pow((C_Ventricle.y - C_Atrium.y), 2) + pow((C_Ventricle.z - C_Atrium.z), 2));
		float l_LVtoAR = sqrt(pow((C_Ventricle.x - C_Aorta1.x), 2) + pow((C_Ventricle.y - C_Aorta1.y), 2) + pow((C_Ventricle.z - C_Aorta1.z), 2));
		PointXYZ unit_vector;
		float T = 13.0;
		unit_vector.x = (C_Aorta1.x - C_Ventricle.x) / l_LVtoAR;
		unit_vector.y = (C_Aorta1.y - C_Ventricle.y) / l_LVtoAR;
		unit_vector.z = (C_Aorta1.z - C_Ventricle.z) / l_LVtoAR;
		if (l_LVtoLA > l_LVtoAR) {
			C_AorticValve.x = C_Aorta1.x + T*unit_vector.x;
			C_AorticValve.y = C_Aorta1.y + T*unit_vector.y;
			C_AorticValve.z = C_Aorta1.z + T*unit_vector.z;
		}
		else {
			C_AorticValve.x = C_Aorta1.x - T*unit_vector.x;
			C_AorticValve.y = C_Aorta1.y - T*unit_vector.y;
			C_AorticValve.z = C_Aorta1.z - T*unit_vector.z;
		}
	}
	
	printf("C_MitralValve     x : %f  y : %f  z : %f\n", C_MitralValve.x, C_MitralValve.y, C_MitralValve.z);
	printf("C_AorticValve     x : %f  y : %f  z : %f\n", C_AorticValve.x, C_AorticValve.y, C_AorticValve.z);



	//set cuboid of mitralValve
	for (size_t f = 0; f < cloud.size(); f++) {
		if (timing[f][0] == 1 && timing[f][1] == 2) {
			for (size_t n = 0; n < cloud[f]->size(); n++) {
				if ((C_MitralValve.x - l / 2.0 < cloud[f]->points[n].x&&cloud[f]->points[n].x < C_MitralValve.x + l / 2.0) && (C_MitralValve.y - l / 2.0 < cloud[f]->points[n].y&&cloud[f]->points[n].y < C_MitralValve.y + l / 2.0) && (C_MitralValve.z - l / 2.0 < cloud[f]->points[n].z&&cloud[f]->points[n].z < C_MitralValve.z + l / 2.0)) {
					PointXYZRGB p;
					p.x = cloud[f]->points[n].x;
					p.y = cloud[f]->points[n].y;
					p.z = cloud[f]->points[n].z;
					p.r = cloud[f]->points[n].r;
					p.g = cloud[f]->points[n].g;
					p.b = cloud[f]->points[n].b;
					for (size_t F = 0; F < cloud.size(); F++) {
						cloud_MitralValve[F]->push_back(p);
					}
				}
			}

		}
		//cloud_MitralValve[f]->push_back(C_MitralValve);
		//cerr << "cloud_MitralValve[" << f << "]->size() : " << cloud_MitralValve[f]->size() << endl;
	}
	//set cuboid of aorticValve
	for (size_t f = 0; f < cloud.size(); f++) {
		if (timing[f][0] == 1 && timing[f][1] == 2) {
			for (size_t n = 0; n < cloud[f]->size(); n++) {
				if ((C_AorticValve.x - l / 2.0 < cloud[f]->points[n].x&&cloud[f]->points[n].x < C_AorticValve.x + l / 2.0) && (C_AorticValve.y - l / 2.0 < cloud[f]->points[n].y&&cloud[f]->points[n].y < C_AorticValve.y + l / 2.0) && (C_AorticValve.z - l / 2.0 < cloud[f]->points[n].z&&cloud[f]->points[n].z < C_AorticValve.z + l / 2.0)) {
					PointXYZRGB p;
					p.x = cloud[f]->points[n].x;
					p.y = cloud[f]->points[n].y;
					p.z = cloud[f]->points[n].z;
					p.r = cloud[f]->points[n].r;
					p.g = cloud[f]->points[n].g;
					p.b = cloud[f]->points[n].b;
					cloud_AorticValve[f]->push_back(p);
					for (size_t F = 0; F < cloud.size(); F++) {
						cloud_AorticValve[F]->push_back(p);
					}
				}
			}

		}
		//cloud_AorticValve[f]->push_back(C_AorticValve);
		//cerr << "cloud_AorticValve[" << f << "]->size() : " << cloud_AorticValve[f]->size() << endl;
	}

	for (size_t f = 0; f < cloud.size(); f++) {
		cloud_MitralValve[f]->push_back(C_MitralValve);
		cloud_AorticValve[f]->push_back(C_AorticValve);
	}
}

void CSVoutMitralValveCentroid(PointCloud<PointXYZRGB>::Ptr Mitral_Valve, int filenumber, int fileindex)
{
	char filename[256];
	sprintf_s(filename, "C:\\Users\\guest1\\Desktop\\心臓データ\\valveEstimation\\MitralValve\\mitralValve%d_%d.csv", filenumber, fileindex);
	ofstream save(filename);

	int index_last = Mitral_Valve->size() - 1;

	char bufx[11], bufy[11], bufz[11];
	snprintf(bufx, 10, "%.2f", Mitral_Valve->points[index_last].x); save << bufx; save << ',';
	snprintf(bufy, 10, "%.2f", Mitral_Valve->points[index_last].y); save << bufy; save << ',';
	snprintf(bufz, 10, "%.2f", Mitral_Valve->points[index_last].z); save << bufz; save << endl;
}

void CSVoutAorticValveCentroid(PointCloud<PointXYZRGB>::Ptr Aortic_Valve, int filenumber, int fileindex)
{
	char filename[256];
	sprintf_s(filename, "C:\\Users\\guest1\\Desktop\\心臓データ\\valveEstimation\\AorticValve\\aorticValve%d_%d.csv", filenumber, fileindex);
	ofstream save(filename);

	int index_last = Aortic_Valve->size() - 1;

	save << setprecision(5) << setiosflags(ios::fixed);
	save << setw(10) << Aortic_Valve->points[index_last].x << "," << setw(10) << Aortic_Valve->points[index_last].y << "," << setw(10) << Aortic_Valve->points[index_last].z << endl;
	save.close();
	/*char bufx[12], bufy[12], bufz[12];
	snprintf(bufx, 11, "%.6f", Aortic_Valve->points[index_last].x); save << bufx; save << ',';
	snprintf(bufy, 11, "%.6f", Aortic_Valve->points[index_last].y); save << bufy; save << ',';
	snprintf(bufz, 11, "%.6f", Aortic_Valve->points[index_last].z); save << bufz; save << endl;*/
}

void averagePointCloud(vector<PointCloud<PointXYZRGB>::Ptr> cloud, PointCloud<PointXYZRGBA>::Ptr &cloud_ave)
{
	vector<vector<vector<vector<int>>>> Pixel;
	Pixel.resize(cloud.size());
	for (int F = 0; F < cloud.size(); F++) {
		Pixel[F].resize(levs);
		for (int Z = 0; Z < levs; Z++) {
			Pixel[F][Z].resize(rows);
			for (int Y = 0; Y < rows; Y++) {
				Pixel[F][Z][Y].resize(cols);
				for (int X = 0; X < cols; X++) {
					Pixel[F][Z][Y][X] = 0;
				}
			}
		}
	}

	for (int F = 0; F < cloud.size(); F++) {
		for (size_t n = 0; n < cloud[F]->size(); n++) {
			int X = (int)cloud[F]->points[n].x;
			int Y = (int)cloud[F]->points[n].y;
			int Z = (int)cloud[F]->points[n].z;
			Pixel[F][Z][Y][X] = cloud[F]->points[n].r;
		}
	}

	for (int Z = 0; Z < levs; Z++) {
		for (int Y = 0; Y < rows; Y++) {
			for (int X = 0; X < cols; X++) {
				PointXYZRGBA p;
				p.x = X;
				p.y = Y;
				p.z = Z;
				float pix = 0;
				for (int F = 0; F < cloud.size(); F++) {
					pix += (float)Pixel[F][Z][Y][X];
				}
				pix /= cloud.size();
				p.r = (int)pix;
				p.g = (int)pix;
				p.b = (int)pix;

				if (p.r < 50 ) {
					p.a = 0;
				}
				else {
					p.a = 255;
				}

				cloud_ave->push_back(p);
			}
		}
	}
}

void oneSlideView(visualization::PCLVisualizer *viewer, vector<vector<vector<PointCloud<PointXYZRGB>::Ptr>>> &cloud_smallgrid, int i, int j, int k)
{
	/*for (int Y = 0; Y < rows / div3; Y++) {
	for (int Z = 0; Z < levs / div3; Z++) {
	stringstream ss_line;
	ss_line << "cloud_info_" << Y*rows / div3 + Z;
	viewer->removePointCloud(ss_line.str());

	}
	}*/

	PointXYZ p;
	p.x = i*div3*PixelSpacingX;
	p.y = j*div3*PixelSpacingY;
	p.z = k*div3*PixelSpacingZ;
	PointCloud<PointXYZ>::Ptr P(new PointCloud<PointXYZ>);
	P->push_back(p);

	viewer->addPointCloud(P, "P");
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 15, "P");

	/*for (int Y = 0; Y < rows / div3; Y++) {
	for (int Z = 0; Z < levs / div3; Z++) {
	stringstream ss_line;
	ss_line << "cloud_infoYZ_" << Y*rows / div3 + Z;
	viewer->addPointCloud(cloud_smallgrid[Z][Y][i], ss_line.str());
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss_line.str());
	}
	}

	for (int X = 0; X < cols / div3; X++) {
	for (int Z = 0; Z < levs / div3; Z++) {
	stringstream ss_line;
	ss_line << "cloud_infoXZ_" << X*cols / div3 + Z;
	viewer->addPointCloud(cloud_smallgrid[Z][j][X], ss_line.str());
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss_line.str());
	}
	}*/

}

void createOneSlice(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr &cloud_filtered, int i, int j, int k, int theta)
{
	double theta1_rad = theta / 360.0 * 2 * M_PI;
	double theta2_rad = M_PI_2 - theta / 360.0 * 2 * M_PI;
	double x1, x2;
	double y1, y2;
	double b = 1 * (PixelSpacingX + PixelSpacingY) / 2.0;

	for (size_t n = 0; n < cloud->size(); n++) {
		y1 = tanf(theta1_rad)*(cloud->points[n].x - i*PixelSpacingX) + j*PixelSpacingY;
		y2 = -tanf(theta2_rad)*(cloud->points[n].x - i*PixelSpacingX) + j*PixelSpacingY;
		if (cloud->points[n].y >(y1 - b) && cloud->points[n].y<(y1 + b) || cloud->points[n].y>(y2 - b) && cloud->points[n].y < (y2 + b)) {
			cloud_filtered->push_back(cloud->points[n]);
		}
		x1 = 1.0 / tanf(theta1_rad)*(cloud->points[n].y - j*PixelSpacingY) + i*PixelSpacingX;
		x2 = -1.0 / tanf(theta2_rad)*(cloud->points[n].y - j*PixelSpacingY) + i*PixelSpacingX;
		if (cloud->points[n].x >(x1 - b) && cloud->points[n].x<(x1 + b) || cloud->points[n].x>(x2 - b) && cloud->points[n].x < (x2 + b)) {
			cloud_filtered->push_back(cloud->points[n]);
		}
	}

	//for (size_t n = 0; n < cloud->size(); n++) {
	//	//printf("%d\n", n);
	//	if (cloud->points[n].x >(i)*div3*PixelSpacingX && cloud->points[n].x < (i + 1)*div3*PixelSpacingX || cloud->points[n].y >(j-1)*div3*PixelSpacingY && cloud->points[n].y < (j + 1)*div3*PixelSpacingY) {
	//		//printf("a\n");
	//		cloud_filtered->push_back(cloud->points[n]);
	//	}
	//}
}