#include "FlyCapture2.h"
#include "BusManager.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include "TcpClient/TcpClient.h"
#include "T0R0Vision.h"


using namespace FlyCapture2;

int main()
{
  T0R0Vision *vision = new T0R0Vision();
  BusManager *bus =new BusManager();

  cv::Size smallimg(640,512);
  cv::Size window(1280,1024);
  cv::Mat win_mat(window, CV_8UC3);
  cv::Mat matPG_small(smallimg, CV_8UC3);
  cv::Mat Armbuff;
  cv::Mat Navbuff;
  cv::Mat cache;
  cv::Mat matARM;
  cv::Mat matNav;
  cv::Mat matARM_small;
  cv::Mat matNav_small;

  std::cout << "T0-R0 camera display and stream" << " ";

  // ****************************  Webcam setup section *******************************

  cv::VideoCapture cap(0);
  cv::VideoCapture cap1(1);


   //if not success, exit program
   if (cap.isOpened() == false)
   {
    std::cout << "Cannot open the ARM camera" << std::endl;
    //return -1;
   }
   if (cap1.isOpened() == false)
   {
    std::cout << "Cannot open the Chassis camera" << std::endl;
    //return -1;
   }

// ************** Pointgrey setup **************************************+

    Error error;
    Camera camera;
    CameraInfo camInfo;
    Error bus1 = bus->ForceAllIPAddressesAutomatically();
    // Connect the camera
    error = camera.Connect( 0 );
    if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to connect to camera" << std::endl;
        return false;
    }
    // Get the camera info and print it out
    error = camera.GetCameraInfo( &camInfo );
    if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to get camera info from camera" << std::endl;
        return false;
    }
    std::cout << camInfo.vendorName << " "
              << camInfo.modelName << " "
              << camInfo.serialNumber << std::endl;

    error = camera.StartCapture();
    if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
    {
        std::cout << "Bandwidth exceeded" << std::endl;
        return false;
    }
    else if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to start image capture" << std::endl;
        return false;
    }
cv::VideoWriter out("appsrc ! videoconvert ! videoscale ! video/x-raw,format=YUY2,width=1280,height=1024, framerate=30/1 ! jpegenc quality=50 ! rtpjpegpay ! udpsink host=10.0.0.105 port=50215 ", 1800,0,30, cv::Size(1280,1024), true);

if (out.isOpened()){
	puts("Pipeline Opened");
}

else {
	puts("Pipeline Broken");
}
char key = 0;
int lost =0;
// ++++++++++++ LOOOOOOOOP +++++++++++
while(key != 'q'){
  {

// +++++++++++ Pointgrey acquire ++++++++++++
	Image raw;
	Error error = camera.RetrieveBuffer(&raw);
	if (error != PGRERROR_OK){
		//std::cout << "network loss frame" << std::endl;
                lost++;
		continue;
	}
//	printf("Frames lost: %d \n", lost);
	Image rgb;

  raw.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgb );

	unsigned int row = (double)rgb.GetReceivedDataSize()/(double)rgb.GetRows();

	cv::Mat image = cv::Mat(rgb.GetRows(), rgb.GetCols(), CV_8UC3, rgb.GetData(),row);

  cv::Mat matPG = cv::Mat(rgb.GetRows(), rgb.GetCols(), CV_8UC3, rgb.GetData(),row);

  resize(matPG, matPG_small, smallimg, cv::INTER_CUBIC);

  // Webcam buffering and collecting +++++++++++++++++++
        if(cap.read(Armbuff)) // get a new frame from camera
	      {
        Armbuff.copyTo(matARM);
        //cap >> matARM;
	      }
        if(cap1.read(Navbuff)) // get a new frame from camera
        {
          Navbuff.copyTo(matNav);
          //cap >> matNav;
        }
  	cv::resize(matARM, matARM_small, smallimg, cv::INTER_CUBIC );
    cv::resize(matNav, matNav_small, smallimg, cv::INTER_CUBIC );
// +++++++++++ Case of use  ++++++++++++

  cache = vision->findCache(matPG);
  cv::resize(cache, cache, smallimg, cv::INTER_CUBIC );
//    ++++++++++++  Rectangles assembly ++++++++++++++++++++++

        matPG_small.copyTo(win_mat(cv::Rect(0,0,640,512)));
        matARM_small.copyTo(win_mat(cv::Rect(0,512,640,512)));
        matNav_small.copyTo(win_mat(cv::Rect(640,0,640,512)));
        cache.copyTo(win_mat(cv::Rect(640,512,640,512)));
        //cv::imshow("image", win_mat);
        out.write(win_mat);
	      key= cv::waitKey(30);
}

 error = camera.StopCapture();
    if ( error != PGRERROR_OK )
    {
        // This may fail when the camera was removed, so don't show
        // an error message
    }

    camera.Disconnect();
    cap.release();
    cap1.release();

    return 0;
}
