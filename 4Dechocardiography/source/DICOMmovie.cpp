#include "stdafx.h"

//DCMTK include
#include <dcmtk/dcmdata/dctk.h>
#include <dcmtk/dcmimgle/dcmimage.h>
#include <dcmtk/dcmimage/diregist.h>

//OpenCV include
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

//stream include
#include <iostream>

//others
#include <stdio.h>

using namespace cv;
using namespace std;

#define thresh 0

int main(void)
{
	OFLog::configure(OFLogger::INFO_LOG_LEVEL);

	DcmFileFormat dcmfile;
	DcmDataset *dataset;

	const Uint8 *frames = NULL;
	//const char *frames = NULL;

	const unsigned long elemNumber = 6;
	Uint32 Len;
	unsigned long flags = 0, fstart = 0, fcount = 1;

	//DcmStack stack;
	//DcmObject *obj;
	DcmElement *element;

	if (dcmfile.loadFile("C:\\Users\\gen\\Desktop\\\Echocardiography\\3DSlicer\\A_11Jul2017_1.2.840.113663.1500.1.327662638.3.1.20170711.180346.375_.dcm").bad()) {
		printf("DcmFileFormat:loadFile() failed.\n");
		return 1;
	}
	dataset = dcmfile.getDataset();

	if (!dataset->findAndGetElement(DcmTagKey(0x7fe0, 0x0010), element).good()) {
		printf("frames couldn't be aquired.\n");
		return -1;
	}

	DcmOtherByteOtherWord *pixdata = (DcmOtherByteOtherWord*)element;
	Uint8 *pixels2;
	pixdata->getUint8Array(pixels2);

	/*
	int N = 0;
	while (N!=10309580) {
		if (*(pixels2 + N) < 200) {
			*(pixels2 + N) = 0;
		}
		N++;
	}
	*/

	int n = 0;
	while (n!=61341696) {
		if (n % 1 == 0) {
			//if (*(pixels2 + n) != 0) {
				//printf("pixels2 = %d   n = %d\n", *(pixels2 + n), n);
			printf("%d ", *(pixels2 + n));
			//}
		}
		
		Mat frame = Mat(Size(192, 128), CV_8UC1, pixels2 + n);

		namedWindow("DICOMtoPNG", WINDOW_AUTOSIZE);
		imshow("DICOMtoPNG", frame);
		waitKey(0);
		destroyAllWindows();
		
		
		n+=(192*128);
		
	}


	/*
	Mat frame = Mat(Size(383, 383), CV_8UC1, pixels2);
	namedWindow("DICOMtoPNG", WINDOW_AUTOSIZE);
	imshow("DICOMtoPNG", frame);
	waitKey(0);
	destroyAllWindows();
	*/

	/*
	if (!dataset->findAndGetElements(DcmTagKey(0x200d, 0x3cfb), stack).good()) {
		printf("frames couldn't be aquired.\n");
		return -1;
	}

	DcmOtherByteOtherWord *headerdata = (DcmOtherByteOtherWord*)stack.elem(elemNumber);

	OFString headerArray;
	headerdata->getOFStringArray(headerArray);
	*/

	//uchar *pixels = (uchar*)pixels2;
	//printf("%s\n", &pixels);


	//Len = stack.elem(elemNumber)->getLength();
	//printf("Length = %d\n", Len);

	//DicomImage image(stack.elem(elemNumber), EXS_LittleEndianExplicit, flags, fstart, fcount);
	//DicomImage image(element, EXS_LittleEndianExplicit, flags, fstart, fcount);

	/*
	if (image.getStatus() == EIS_Normal) {
		do {
			int X = image.getFirstFrame();
			//DCMIMGLE_INFO("processing frame " << X + 1 << " to " << X + frames->getFrameCount());
			uchar *pixelData = (uchar*)(image.getOutputData(8));
			if (pixelData != NULL) {
				Mat frame = Mat(Size(int(image.getWidth()), int(image.getHeight())), CV_8UC1, pixelData);

				//ofstream ofs("frame.csv");
				//ofs << format(frame, Formatter::FMT_CSV) << endl;

				namedWindow("DICOMtoPNG", WINDOW_AUTOSIZE);
				imshow("DICOMtoPNG", frame);
				waitKey(0);
				destroyAllWindows();

			}
		} while (image.processNextFrames());
	}
	*/

	


	//delete frames;

	/*
	if (frames->getStatus() == EIS_Normal) {
		
		do {
			int X = frames->getFirstFrame();
			DCMIMGLE_INFO("processing frame " << X + 1 << " to " << X + frames->getFrameCount());
			uchar *pixelData = (uchar*)(frames->getOutputData(8));
			if (pixelData != NULL) {
				Mat frame = Mat(Size(int(frames->getWidth()), int(frames->getHeight())), CV_8UC3, pixelData);

				//ofstream ofs("frame.csv");
				//ofs << format(frame, Formatter::FMT_CSV) << endl;

				namedWindow("DICOMtoPNG", WINDOW_AUTOSIZE);
				imshow("DICOMtoPNG", frame);
				waitKey(0);
				destroyAllWindows();

			}
	
		} while (frames->processNextFrames());
		
	}
	

	delete frames;
	*/

	return 0;
}