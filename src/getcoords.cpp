#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <fstream>      // std::ofstream
#include <cv.h>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include <cstdio>
#include <ARToolKitPlus/TrackerMultiMarker.h>
#include <math.h>
#include <cstdlib>
#include <unistd.h>
#include <new>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
using namespace std;
using namespace cv;
using ARToolKitPlus::TrackerMultiMarker;

int markerWidth = 28;


int getNumDetected(IplImage *img, TrackerMultiMarker *tracker,	int width, int height) {
	int numDetected = 0;
	IplImage *greyImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

	IplImage *tempImg = cvCreateImage(cvSize(width, height), img->depth, 1);
	cvCvtColor(img, tempImg, CV_RGB2GRAY);
	cvAdaptiveThreshold(tempImg, greyImg, 255.0, CV_ADAPTIVE_THRESH_GAUSSIAN_C,
		CV_THRESH_BINARY, 31);
	numDetected = tracker->calc((unsigned char*) greyImg->imageData);

	IplImage* new_img = cvCreateImage(cvSize(640, 480), greyImg->depth, greyImg->nChannels);
	cvResize(greyImg, new_img);

	cvShowImage("Automated Arm", new_img);
	cvWaitKey(1); // Wait for image to be rendered on screen. If not included, no image is shown.
	
	cvReleaseImage(&new_img);
	cvReleaseImage(&greyImg);
	cvReleaseImage(&tempImg);
	return numDetected;
//
}

void getMarkerPosition(TrackerMultiMarker* tracker, int index,
	Mat &pose) {
	ARToolKitPlus::ARMarkerInfo markerInfo = tracker->getDetectedMarker(index);
	ARFloat nOpenGLMatrix[16];
	ARFloat patternCentre[2] = { 0.0f, 0.0f };
	tracker->calcOpenGLMatrixFromMarker(&markerInfo, patternCentre, markerWidth,
		nOpenGLMatrix);
	Mat tmp = Mat(4, 4, CV_32FC1, (float *) nOpenGLMatrix);
	pose.at<float>(0, 0) = tmp.at<float>(3, 0);
	pose.at<float>(1, 0) = tmp.at<float>(3, 1);
	pose.at<float>(2, 0) = tmp.at<float>(3, 2);
}

int main(int argc, char** argv) {
	// Open capture
	VideoCapture cap;
	Mat img;
	cap.open(1);
	if (!cap.isOpened()) {
			std::cout << "Could not initialize capturing...\n" << std::endl;
			exit(EXIT_FAILURE);
		}
	cap.read(img);



	int width = img.cols, height = img.rows;
	int marker0 = 495, marker1 = 493, marker2 = 492, marker3=494;
	Mat pose0 = (Mat_<float>(8,1) << 0, 0, 0, 0, 0, 0, 1, 1);
	Mat pose1 = (Mat_<float>(8,1) << 0, 0, 0, 0, 0, 0, 1, 1); 
	Mat pose2 = (Mat_<float>(8,1) << 0, 0, 0, 0, 0, 0, 1, 1); 
	Mat pose3 = (Mat_<float>(8,1) << 0, 0, 0, 0, 0, 0, 1, 1); 

	
	IplImage* img_proc = cvCreateImage(cvSize(width, height),
		IPL_DEPTH_8U, img.channels());
// Initialise Artoolkitplus
	TrackerMultiMarker* tracker = new TrackerMultiMarker(width, height, 8, 6, 6, 6, 0);
	tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);

	// load a camera file.
	if (!tracker->init("../src/Logitech_Notebook_Pro.cal",	"../src/markerboard_480-499.cfg", 1.0f, 1000.0f))
	{
		printf("ERROR: init() failed\n");
		exit(EXIT_FAILURE);
	}

	// tracker->getCamera()->printSettings();

	// the marker in the BCH test image has a thin border...
	tracker->setBorderWidth(0.125);

	// set a threshold. alternatively we could also activate automatic thresholding
	tracker->setThreshold(120);

	// let's use lookup-table undistortion for high-speed
	// note: LUT only works with images up to 1024x1024
	tracker->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);

	// switch to simple ID based markers
	// use the tool in tools/IdPatGen to generate markers
	tracker->setMarkerMode(ARToolKitPlus::MARKER_ID_SIMPLE);




// Start fifo
	int fd;
	char * myfifo = "/tmp/autoarm";
/* create the FIFO (named pipe) */
	mkfifo(myfifo, 0666);
	fd = open(myfifo, O_WRONLY);

	while (1){
		cap.read(img);
		img_proc->imageData = (char *) img.data;
		int num = getNumDetected(img_proc, tracker, width,	height);
		cout << num << " marker(s) detected"<< endl;

		if (num == 4) {
			for (int i = 0; i < num; i ++) {
				int markerID = tracker->getDetectedMarker(i).id;

				if ( markerID == marker0 ) {
					getMarkerPosition(tracker, i, pose0); 
				} else if ( markerID == marker1 ) {
					getMarkerPosition(tracker, i, pose1); 
				}else if ( markerID == marker2 ) {
					getMarkerPosition(tracker, i, pose2); 
				}else if ( markerID == marker3 ) {
					getMarkerPosition(tracker, i, pose3); 
				}

			}
			float out[13] = {pose0.at<float>(0, 0),pose0.at<float>(1, 0),pose0.at<float>(2, 0),
				pose1.at<float>(0, 0),pose1.at<float>(1, 0),pose1.at<float>(2, 0),
				pose2.at<float>(0, 0),pose2.at<float>(1, 0),pose2.at<float>(2, 0),
				pose3.at<float>(0, 0),pose3.at<float>(1, 0),pose3.at<float>(2, 0), pose3.at<float>(5, 0)};
			write(fd, out, 13*sizeof(float));
			usleep(100000);
		}
	}
	cap.release();
	close(fd);
}
