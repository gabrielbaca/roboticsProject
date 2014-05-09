#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <errno.h>

/* Definifición del tamaño de la imagen */
#define imageHeight 240
#define imageWidth 320

using std::cout;
using std::cin;
using std::endl;
using namespace cv;

/* Declaración de variables globales */
int coordinateX = 0, coordinateY = 0;
int hueClick = 0, saturationClick = 0, valueClick = 0;
int fd;		// Variable que contendrá descriptor de archivo
Mat currentImage = Mat(imageHeight, imageWidth, CV_8UC3);
Mat hsvImage = Mat(imageHeight, imageWidth, CV_8UC3);


/* Prototipo de funciones */
void mouseCoordinates(int event, int x, int y, int flags, void* param);
Mat correctGamma(Mat &img, double gamma);
void complementImage(const Mat &sourceImage, Mat &destinationImage);
void thresholdImage(const Mat &sourceImage, Mat &destinationImage);
void deteccion_fuego(Mat frame);
void print();
void printLocation(char);

int main()
{
	/* Configuración de la camara*/
	VideoCapture cameraFeed;
	cameraFeed.set(CV_CAP_PROP_FRAME_WIDTH, imageWidth);
	cameraFeed.set(CV_CAP_PROP_FRAME_HEIGHT, imageHeight);
	cameraFeed.open(0);

	/* Declaración de variables */
	vector <vector<Point> > contours;
	vector <Vec4i> hierarchy;
	char key = ' ';
	int thresh = 100;
	char freeze = 0;
	char location = '0';
	Rect aux;
	

	/* Declaración de matrices */
	Mat cannyOutput;
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1,-1));		// Elemento que especifica el tamaño de las operaciones morfológicas
	Mat gammaImage = Mat(imageHeight, imageWidth, CV_8UC3);							
	Mat complementedImage = Mat(imageHeight, imageWidth, CV_8UC3);
	Mat binImageYellow = Mat(imageHeight, imageWidth, CV_8UC1);
	Mat binImageRed = Mat(imageHeight, imageWidth, CV_8UC1);
	Mat binImagePink = Mat(imageHeight, imageWidth, CV_8UC1);
	Mat binImage = Mat(imageHeight, imageWidth, CV_8UC1);
	Mat binImageSecondPink = Mat(imageHeight, imageWidth, CV_8UC1);
	Mat filledImage = Mat(imageHeight, imageWidth, CV_8UC1);
	Mat grayImage = Mat(imageHeight, imageWidth, CV_8UC3);

	/* Configuración del puerto serial */
	if ((fd = serialOpen ("/dev/ttyAMA0", 9600)) < 0)
	{
	    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
	    return 1 ;
	}

	/* Configuración antes de comenzar el procesamiento */
	printf("\033[2J\033[1;1H");		// Se limpia la consola
	RNG rng(12345);					// Declaración de generador aleatorio de números, con semilla 12345

	while(key != 27)
	{
		fflush(stdout);

		if(key == 'f')	// En caso de leer una 'f', se congela el procesamiento
		{
			freeze = !freeze;
			key = '0';
		}
		if(!freeze )
		{
			cameraFeed >> currentImage;		// Se obtiene el frame actual de la cámara

			/* Procesamiento de imagen */			
			gammaImage = correctGamma(currentImage, 0.15);		// Se hace una corrección de Gamma sobre la imagen
			bitwise_not(gammaImage, complementedImage);			// Se invierten los colores de la imagen
			cvtColor(complementedImage, hsvImage, CV_BGR2HSV);	// Se transforma al espacio de color HSV
			inRange(hsvImage, Scalar(15, 0, 245), Scalar(40, 20, 255), binImageSecondPink);	// Se filtra el color rosa
			inRange(hsvImage, Scalar(0, 0, 253), Scalar(1, 1, 255), binImageYellow); 		// Se filtra el color amarillo
			inRange(hsvImage, Scalar(25, 0, 253), Scalar(35, 25, 255), binImagePink); 		// Se filtra el color rosa
			inRange(hsvImage, Scalar(120, 30, 108), Scalar(150, 50, 115), binImageRed); 	// Se filtra el color rojo
			
			/* Unión de las imágenes filtradas*/
			bitwise_or(binImageYellow, binImagePink, binImage);			
			bitwise_or(binImage, binImageRed, binImage);				
			bitwise_or(binImage, binImageSecondPink, binImage);

			bitwise_not(binImage, binImage); 	// Inversión de colores
			morphologyEx(binImage, filledImage, MORPH_OPEN, element);	// Apertura para eliminar el ruido
			dilate(binImage, binImage, element);	// Dilatación para que se detecte mejor el objeto
			Canny(filledImage, cannyOutput, thresh, thresh * 2, 3);	// Resaltar los contornos

			/* Detección de contornos */
			findContours(cannyOutput, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0)); // Detección de contornos
			
			if(contours.size() > 0)
			{
				Moments mu;
				Point2f mc;
				vector<Rect> boundRect(contours.size());	// Contiene rectángulo a partir de polígono
				vector <vector<Point> > contours_poly( contours.size() );	// Contiene los polígonos aproximados
				Mat drawing = Mat::zeros( cannyOutput.size(), CV_8UC3);		// Matriz que dibuja los contornos
				for(unsigned int i = 0; i < contours.size(); i++ )
				{
					mu = moments(contours[i]);		// Cálculo de momentos de Hu
					mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);	// Cálculo de centro de masa
					int center = mc.x;

					if(mc.y > 30) // Ignora la sección superior por 30 píxeles
					{
						double a = contourArea(contours[i], false);	// Obtiene área
						if(a > 0)
						{	
							Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
							drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());	// Dibujo de contornos sobre la matriz drawing	
							approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true);		// Aproximación a polígonos
							boundRect[i] = boundingRect(Mat(contours_poly[i]));
							
							if (boundRect[i].width / boundRect[i].height >= 0)
							{
								/* Detección de la región en la que se encuentra la flama */
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
		printLocation(location);	// Se envía la región al PIC32
		key = waitKey(15);			// Se hace un delay de 15 ms, y se revisa el buffer de teclado
	}
	serialClose(fd);				// Se cierra puerto serial
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

/* Función que altera el Gamma */
Mat correctGamma(Mat &img, double gamma) 
{
	double inverse_gamma = 1.0 / gamma;
 
	Mat lut_matrix(1, 256, CV_8UC1);
	uchar * ptr = lut_matrix.ptr();
	for(int i = 0; i < 256; i++)
		ptr[i] = (int)(pow((double) i / 255.0, inverse_gamma) * 255.0);
 
	Mat result;
	LUT(img, lut_matrix, result);
 
	return result;
}

/* Envío de información al PIC32 y a la consola */
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
