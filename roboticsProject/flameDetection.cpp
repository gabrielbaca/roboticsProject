#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <errno.h>
#define imageHeight 240
#define imageWidth 320

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
int fd;
RNG rng(12345);
Mat currentImage = Mat(imageHeight, imageWidth, CV_8UC3);
Mat hsvImage = Mat(imageHeight, imageWidth, CV_8UC3);

void mouseCoordinates(int event, int x, int y, int flags, void* param);
Mat correctGamma(Mat &img, double gamma);
void complementImage(const Mat &sourceImage, Mat &destinationImage);
void thresholdImage(const Mat &sourceImage, Mat &destinationImage);
void deteccion_fuego(Mat frame);
void print();
void printLocation(char);

int main()
{
	VideoCapture cameraFeed;
	cameraFeed.set(CV_CAP_PROP_FRAME_WIDTH, imageWidth);
	cameraFeed.set(CV_CAP_PROP_FRAME_HEIGHT, imageHeight);
	cameraFeed.open(0);
	vector <vector<Point> > contours;
	vector <Vec4i> hierarchy;
	Rect aux;
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1,-1));

	/* Matrix declaration */
	Mat gammaImage = Mat(imageHeight, imageWidth, CV_8UC3);
	Mat complementedImage = Mat(imageHeight, imageWidth, CV_8UC3);
	Mat binImageYellow = Mat(imageHeight, imageWidth, CV_8UC1);
	Mat binImageRed = Mat(imageHeight, imageWidth, CV_8UC1);
	Mat binImagePink = Mat(imageHeight, imageWidth, CV_8UC1);
	Mat binImage = Mat(imageHeight, imageWidth, CV_8UC1);
	Mat binImagePaul = Mat(imageHeight, imageWidth, CV_8UC1);
	Mat filledImage = Mat(imageHeight, imageWidth, CV_8UC1);
	Mat grayImage = Mat(imageHeight, imageWidth, CV_8UC3);

	Mat cannyOutput;
	char key = ' ';
	int thresh = 100;
	char freeze = 0;
	char location = '0';
	setMouseCallback("HSV", mouseCoordinates);
	printf("\033[2J\033[1;1H");
	double gammaValue = 0.01;

	if ((fd = serialOpen ("/dev/ttyAMA0", 9600)) < 0)
	{
	    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
	    return 1 ;
	}
	
	if (wiringPiSetup () == -1)
	{
	    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
	    return 1 ;
  	}

	while(key != 27)
	{
		fflush(stdout);

		if(key == 'f')
		{
			freeze = !freeze;
			key = '0';
		}
		if(!freeze )
		{
			cameraFeed >> currentImage;

			/* Procesamiento de imagen */			
			gammaImage = correctGamma(currentImage, 0.15);
			bitwise_not(gammaImage, complementedImage);
			cvtColor(complementedImage, hsvImage, CV_BGR2HSV);
			inRange(hsvImage, Scalar(15, 0, 245), Scalar(40, 20, 255), binImagePaul);
			inRange(hsvImage, Scalar(0, 0, 253), Scalar(1, 1, 255), binImageYellow); 
			inRange(hsvImage, Scalar(25, 0, 253), Scalar(35, 25, 255), binImagePink); 
			inRange(hsvImage, Scalar(120, 30, 108), Scalar(150, 50, 115), binImageRed); 
			bitwise_or(binImageYellow, binImagePink, binImage);
			bitwise_or(binImage, binImageRed, binImage);
			bitwise_or(binImage, binImagePaul, binImage);
			bitwise_not(binImage, binImage);
			morphologyEx(binImage, filledImage, MORPH_OPEN, element);
			dilate(binImage,binImage, element);
			Canny(filledImage, cannyOutput, thresh, thresh * 2, 3);

			/* Detección de contornos */
			findContours(cannyOutput, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
			
			if(contours.size() > 0) 
			{
				Moments mu;
				Point2f mc;
				vector<Rect> boundRect(contours.size());
				vector<vector<Point> > contours_poly( contours.size() );
				Mat drawing = Mat::zeros( cannyOutput.size(), CV_8UC3);
				for(unsigned int i = 0; i < contours.size(); i++ )
				{
					mu = moments(contours[i]);
					mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
					int center = mc.x;
					if(mc.y > 30)
					{
						Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
						drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
	
						double a = contourArea(contours[i], false);
						if(a > 0)
						{	
							drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());			
							approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true);
							boundRect[i] = boundingRect(Mat(contours_poly[i]));
							aux = boundRect[i];
							
							if (boundRect[i].width / boundRect[i].height >= 0)
							{
		
								if(center > (drawing.cols / 5) * 4) 
								{
									fflush (stdout) ;
		      						location = '5';
									serialFlush(fd);
								}

								else if(center > (drawing.cols / 5) *3) 
								{
									fflush (stdout) ;
		      						location = '4';
									serialFlush(fd);
								}
								else if (center > (drawing.cols / 5) * 2) 
								{
									fflush (stdout) ;
		      						location = '3';
									serialFlush(fd);
								}
								else if (center > drawing.cols / 5) 
								{
									fflush (stdout) ;
		      						location = '2';
									serialFlush(fd);
								}
								else 
								{ 
									fflush (stdout) ;
		      						location = '1';
									serialFlush(fd);
								}
							}
							waitKey(50);
						}
					}
				}
			}
			else
			{
				location = '0';
			}	
		}
		printLocation(location);
		key = waitKey(15);
	}
	serialClose(fd);
	return 0;
}

void print()
{
	cout << "H: " << hueClick << ", S: " << saturationClick << ", V: " << valueClick << endl << endl;
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
			printf("\033[2J\033[1;1H");
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

void printLocation (char loc)
{
	printf("\033[2J\033[1;1H");
	if(loc == '0')
	{
		cout << "Flama no encontrada" << endl;
	}
	else if(loc <= '5')
	{
		cout << "Flama en la region " << loc << endl;
	}
	serialPutchar(fd, loc);
}