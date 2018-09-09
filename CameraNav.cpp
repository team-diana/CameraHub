#include "FlyCapture2.h"
#include "BusManager.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include "TcpClient/TcpClient.h"
#include "T0R0Vision.h"
using namespace FlyCapture2;

TcpServer *mouse_server = new TcpServer(MOUSE_PORT);
TcpServer *thresh_server = new TcpServer(THRESH_PORT);
TcpServer *cmds_server = new TcpServer(CMDS_PORT);
T0R0Vision *vision = new T0R0Vision();

int main()
{
double x, y;
int changed;
const std::string SERVER_ADDRESS	{ "10.0.0.10" };
const std::string PORT	{ "50215" };
const std::string MOUSE_PORT	{ "50214" };
const std::string THRESH_PORT	{ "50213" };
const std::string CMDS_PORT	{ "50212" };
const std::string CLIENT_ID		{ "T0-R0 video" };
  cv::Size bigimg(960,768);
  cv::Size smallimg(320,256);
  cv::Size window(1280,1024);
  cv::Mat win_mat(window, CV_8UC3);
  cv::Mat matPG_small(smallimg, CV_8UC3);
  cv::Mat matbig(bigimg, CV_8UC3);
  cv::Mat Armbuff;
  cv::Mat Navbuff;
  int set=0;
  std::cout << "T0-R0 camera display and stream" << " ";
//  scanf("%d", &set);


mouse_server->start16();
thresh_server->start16();
cmds_server->start8();

T0R0Vision *vision = new T0R0Vision();
  // ****************************  Webcam setup section *******************************

  cv::VideoCapture cap(0);
  cv::VideoCapture cap1(1);

  const double fps = cap.get(cv::CAP_PROP_FPS);
  const int width  = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  const int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  const int fourcc = cap.get(cv::CAP_PROP_FOURCC );
  const double fps = cap1.get(cv::CAP_PROP_FPS);
  const int width  = cap1.get(cv::CAP_PROP_FRAME_WIDTH);
  const int height = cap1.get(cv::CAP_PROP_FRAME_HEIGHT);
  const int fourcc = cap1.get(cv::CAP_PROP_FOURCC );
  // printf("%d", cap.get(CV_CAP_PROP_CONVERT_RGB));
  cv::Mat matARM;
  cv::Mat matNav;

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
     cv::Mat matARM_small;
     cv::Mat matNav_small;

// ************** Pointgrey setup **************************************+

    Error error;
    Camera camera;
    CameraInfo camInfo;
    BusManager *bus =new BusManager();
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
cv::VideoWriter out("appsrc ! videoconvert ! videoscale ! video/x-raw,format=YUY2,width=1280,height=1024, framerate=30/1 ! jpegenc quality=50 ! rtpjpegpay ! udpsink host=" + SERVER_ADDRESS + " port="+ PORT +" ", 1800,0,30, cv::Size(1280,1024), true);

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
    if(mouse_server->newDataAvailable())
    {
      x = ((double)(mouse_server->read16());
      y = ((double)(mouse_server->readLast16());
      changed = true;
    }
    if(thresh_server->newDataAvailable())
    {
      thresh = ((double)(thresh_server->readLast16());

      changed = true;
    }
    if(comm_server->newDataAvailable())
    {
      set = ((int)(comm_server->readLast8());

      changed = true;
    }
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
  if (set==1)
  {
   	matbig=matARM;
  }
  else if (set==2)
  {
   	matbig=matPG;
  }
  else if (set==3)
  {
   	matbig=matNav;
  }
  else if (set==4)
  {
   	matbig = vision->findCache(matPG);

  }
  else if (set==5)
  {
   	matbig = vision->findCache(matARM);

  }
  else if (set==6)
  {
   	matbig = vision->findCache(matNav);

  }
  else if (set==7)
  {
   	matbig = vision->findHere(matARM, x, y);
    T0R0Vision::setThreshold(int thresh);

  }
  else if (set==8)
  {
   	matbig = vision->findHere(matPG, x, y);
    T0R0Vision::setThreshold(int thresh);

  }
  else
  {
   	matbig=matbig;
  }

  cv::resize(matbig, matbig, bigimg, cv::INTER_CUBIC );
  cv::resize(logo, logo, smallimg, cv::INTER_CUBIC );

//    ++++++++++++  Rectangles assembly ++++++++++++++++++++++
        matbig.copyTo(win_mat(cv::Rect(  0, 0, 960, 768)));
        matPG_small.copyTo(win_mat(cv::Rect(0,760,320,256)));
        matARM_small.copyTo(win_mat(cv::Rect(320,760,320,256)));
        matNav_small.copyTo(win_mat(cv::Rect(640,760,320,256)));
        logo.copyTo(win_mat(cv::Rect(960,0,320,256)));
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
