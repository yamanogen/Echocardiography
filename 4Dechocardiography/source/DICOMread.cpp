// DICOMread.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"

//DCMTK include
//#include <dcmtk/config/osconfig.h>
//#include <dcmtk/dcmdata/dctk.h>
#include <dcmtk/dcmimgle/dcmimage.h>
#include <dcmtk/dcmimage/diregist.h>

//OpenCV include
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

//pcl include
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>

//stream include
#include <iostream>
#include <fstream>

//others
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

using namespace cv;
using namespace std;

#define thresh 100
#define scaleX 0.25
#define scaleY 0.08
#define scaleZ 0.08

//global var
int user_data;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.addSphere(o, 0.25, "sphere", 0);
	std::cout << "i only run once" << std::endl;

}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:
	user_data++;
}

int main()
{
	/*
	DcmFileFormat dcmfile;
	DcmDataset *dataset;
	Uint16 rows, columns, depths;

	if (dcmfile.loadFile("C:\\Users\\gen\\Desktop\\心臓超音波DICOM\\7\\DICOM\\IM_0005").bad()) {
		printf("DcmFileFormat:loadFile() failed.\n");
		return 1;
	}

	dataset = dcmfile.getDataset();

	if (!dataset->findAndGetUint16(DCM_Columns, columns).good()) {
		printf("Get columns tag failed.\n");
		return 1;
	}

	if (!dataset->findAndGetUint16(DCM_Rows, rows).good()) {
		printf("Get rows tag failed.\n");
		return 1;
	}

	//if (!dataset->findAndGetUint16(DCM_Depth, depths).good()) {
	//	printf("Get rows tag failed.\n");
	//	return 1;
	//}

	printf("cols=%d, rows=%d\n", columns, rows);
	*/
	
	/*
	DicomImage *image = new DicomImage("C:\\Users\\gen\\Desktop\\心臓超音波DICOM\\4\\DICOM\\IM_0001");
	if (image != NULL) {
		if (image->getStatus() == EIS_Normal) {
			uchar *pixelData = (uchar*)(image->getOutputData(8));
			if (pixelData != NULL) {
				Mat img = Mat(Size(int(image->getWidth()), int(image->getHeight())), CV_8UC3, pixelData);
				namedWindow("DICOMtoPNG", WINDOW_AUTOSIZE);
				imshow("DICOMtoPNG", img);
				waitKey(0);
				destroyAllWindows();
			}
		}
	}
	*/

	int n_point = 0;

	OFLog::configure(OFLogger::INFO_LOG_LEVEL);

	DicomImage *frames = new DicomImage("C:\\Users\\gen\\Desktop\\Echocardiography\\3\\DICOM\\IM_0002", CIF_UsePartialAccessToPixelData, 0, 1);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	int W = frames->getWidth();
	int H = frames->getHeight();
	cloud->width = W;
	cloud->height = H;
	cloud->is_dense = true;
	cloud->points.resize(cloud->width*cloud->height);

	/*
	pcl::PointCloud<pcl::PointXYZI> cloud;
	int W = frames->getWidth();
	int H = frames->getHeight();
	cloud.width = W;
	cloud.height = H;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width*cloud.height);
	*/

	/*
	int n = 0;
	int fcount = 10;
	uchar *pixelData = (uchar*)(frames->getOutputData(8));
	while (n != 65000000) {
		printf("%d\n", n / 120000);
		Mat frame = Mat(Size(300, 400), CV_8UC1, pixelData);
		namedWindow("DICOMtoPNG", WINDOW_AUTOSIZE);
		imshow("DICOMtoPNG", frame);
		waitKey(0);
		destroyAllWindows();
		n += 120000 * fcount;
	}
	*/

	
	if (frames->getStatus() == EIS_Normal) {
		do {
			int X = frames->getFirstFrame();
			DCMIMGLE_INFO("processing frame " << X + 1 << " to " << X + frames->getFrameCount());
			uchar *pixelData = (uchar*)(frames->getOutputData(8));
			if (pixelData != NULL) {
				Mat frame = Mat(Size(int(frames->getWidth()), int(frames->getHeight())), CV_8UC1, pixelData);
				//Mat frame = Mat(Size(200, 200), CV_8UC1, pixelData);

				for (int Y = 0; Y < frame.cols; Y++) {
					for (int Z = 0; Z < frame.rows; Z++) {
						int I = frame.at<unsigned char>(Z, Y);
						if (I > thresh) {
							/*
							cloud.points[n_point].x = X;
							cloud.points[n_point].y = Y;
							cloud.points[n_point].z = Z;
							cloud.points[n_point].intensity = I / 255.0;
							n_point++;
							*/
		
							/*
							cloud->points[n_point].x = X;
							cloud->points[n_point].y = Y;
							cloud->points[n_point].z = Z;
							//cloud->points[n_point].intensity = I/255.0;
							n_point++;
							*/

							pcl::PointXYZRGB p;
							p.x = X*scaleX;
							p.y = Y*scaleY;
							p.z = Z*scaleZ;
							/*
							float gray = I / 255.0;
							p.r = gray;
							p.g = gray;
							p.b = gray;
							*/
							p.r = I;
							p.g = I;
							p.b = I;
							//if (I > 240)printf("x=%d y=%d gray=%.1f ", Y, Z, gray);
							cloud->push_back(p);
						}
					}
				}
				//ofstream ofs("frame.csv");
				//ofs << format(frame, Formatter::FMT_CSV) << endl;
				
				//namedWindow("DICOMtoPNG", WINDOW_AUTOSIZE);
				//imshow("DICOMtoPNG", frame);
				//waitKey(0);
				//destroyAllWindows();
				
			}
		} while (frames->processNextFrames());
	}
	

	delete frames;
	
	/*
	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	//blocks until the cloud is actually rendered
	viewer.showCloud(cloud);

	//use the following functions to get access to the underlying more advanced/powerful
	//PCLVisualizer

	//This will only get called once
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	//This will get called once per visualization iteration
	viewer.runOnVisualizationThread(viewerPsycho);
	while (!viewer.wasStopped())
	{
		//you can also do cool processing here
		//FIXME: Note that this is running in a separate thread from viewerPsycho
		//and you should guard against race conditions yourself...
		user_data++;
	}

	*/

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> intensity_distribution(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, intensity_distribution, "sample cloud", 0);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.0001, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	
    return 0;
}

