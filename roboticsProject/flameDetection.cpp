#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <fstream>
#include <iostream>
#include <Windows.h>
#include <queue>
#define imageHeight 480
#define imageWidth 640
using std::cout;
using std::cin;
using std::endl;
using namespace cv;
vector <Mat> hsv_planes;
int coordinateX = 0, coordinateY = 0;
int redClick = 0, blueClick = 0, greenClick = 0;
int hueClick = 0, saturationClick = 0, valueClick = 0;
int maxRed = 0, maxBlue = 0, maxGreen = 0;
int minRed = 0, minBlue = 0, minGreen = 0;
int nClicks = 0;
RNG rng(12345);
Mat currentImage = Mat(imageHeight, imageWidth, CV_8UC3);
Mat gammaImage = Mat(imageHeight, imageWidth, CV_8UC3);
Mat complementedImage = Mat(imageHeight, imageWidth, CV_8UC3);
Mat hsvImage = Mat(imageHeight, imageWidth, CV_8UC3);
Mat binImageYellow = Mat(imageHeight, imageWidth, CV_8UC1);
Mat binImageGreen = Mat(imageHeight, imageWidth, CV_8UC1);
Mat binImageGreenDetail = Mat(imageHeight, imageWidth, CV_8UC1);
Mat binImage = Mat(imageHeight, imageWidth, CV_8UC1);
Mat filledImage = Mat(imageHeight, imageWidth, CV_8UC1);
Mat grayImage = Mat(imageHeight, imageWidth, CV_8UC3);

void mouseCoordinates(int event, int x, int y, int flags, void* param);
Mat correctGamma(Mat &img, double gamma);
void complementImage(const Mat &sourceImage, Mat &destinationImage);
void thresholdImage(const Mat &sourceImage, Mat &destinationImage);
void deteccion_fuego(Mat frame);
void print();

int main()
{
	VideoCapture cameraFeed;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1,-1));
	Mat cannyOutput;
	//Mat kernel = Mat::ones(Size(5, 5), CV_8UC1);
	char key = ' ';
	double gamma = 0.1;
	int counter = 0;
	int thresh = 100;
	char freeze = 0;
	cameraFeed.open(0);
	//namedWindow("Camera Feed");
	//namedWindow("Complemented image");
	namedWindow("HSV");
	//setMouseCallback("Complemented image", mouseCoordinates);
	setMouseCallback("HSV", mouseCoordinates);
	//imshow("Camera Feed", currentImage);
	
	while(key != 27)
	{
	//	system("cls");
	//	print();
		if(key == 'f')
		{
			freeze = !freeze;
			key = '0';
		}
		if(!freeze)
		{
			cameraFeed >> currentImage;
			gammaImage = correctGamma(currentImage, 0.2);
			//complementImage(gammaImage, complementedImage);
			bitwise_not(gammaImage, complementedImage);
			cvtColor(complementedImage, hsvImage, CV_BGR2HSV);
			imshow("HSV", hsvImage);
			//split(hsvImage, hsv_planes);
			inRange(hsvImage, Scalar(85, 230, 185), Scalar(106, 255, 225), binImageYellow);
			inRange(hsvImage, Scalar(90, 250, 5), Scalar(120, 255, 144), binImageGreen);
			bitwise_or(binImageYellow, binImageGreen, binImage);
			//erode(binImage, filledImage, element);
			morphologyEx(binImage, filledImage, MORPH_OPEN, element);
			Canny(filledImage, cannyOutput, thresh, thresh *2, 3);
			findContours(cannyOutput, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

			Mat drawing = Mat::zeros( cannyOutput.size(), CV_8UC3);
			for(unsigned int i = 0; i < contours.size(); i++ )
			{
			  Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
			  drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
			}

			imshow("Binary", filledImage);
			imshow("Contours", drawing);
		}
		key = waitKey(15);
		//Sleep(15);
	}
	return 0;
}
void print()
{
	cout << "X: " << coordinateX << ", Y: " << coordinateY << endl; 
	cout << "R: " << redClick << ", G: " << greenClick << ", B: " << blueClick << endl;
	cout << "H: " << hueClick << ", S: " << saturationClick << ", V: " << valueClick << endl;
}

void mouseCoordinates(int event, int x, int y, int flags, void* param)
{
    switch (event)
    {
	    
	  /*CV_EVENT_MOUSEMOVE - when the mouse pointer moves over the specified window
		CV_EVENT_LBUTTONDOWN - when the left button of the mouse is pressed on the specified window
		CV_EVENT_RBUTTONDOWN - when the right button of the mouse is pressed on the specified window
		CV_EVENT_MBUTTONDOWN - when the middle button of the mouse is pressed on the specified window
		CV_EVENT_LBUTTONUP - when the left button of the mouse is released on the specified window
		CV_EVENT_RBUTTONUP - when the right button of the mouse is released on the specified window
		CV_EVENT_MBUTTONUP - when the middle button of the mouse is released on the specified window */

        case CV_EVENT_LBUTTONDOWN:
            coordinateX = x;
            coordinateY = y;
            redClick = currentImage.at<Vec3b>(y, x)[2];
	        greenClick = currentImage.at<Vec3b>(y, x)[1];
	        blueClick = currentImage.at<Vec3b>(y, x)[0];

			hueClick = hsvImage.at<Vec3b>(y, x)[0];
	        saturationClick = hsvImage.at<Vec3b>(y, x)[1];
	        valueClick = hsvImage.at<Vec3b>(y, x)[2];
			print();
            break;
        case CV_EVENT_RBUTTONDOWN:
			system("cls");
            break;
    }
}

Mat correctGamma(Mat &img, double gamma) {
	double inverse_gamma = 1.0 / gamma;
 
	Mat lut_matrix(1, 256, CV_8UC1);
	uchar * ptr = lut_matrix.ptr();
	for(int i = 0; i < 256; i++)
	ptr[i] = (int)(pow((double) i / 255.0, inverse_gamma) * 255.0);
 
	Mat result;
	LUT(img, lut_matrix, result);
 
	return result;
}