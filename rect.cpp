#include "FlyCapture2.h"
#include "BusManager.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
using namespace FlyCapture2;

#define SERVER_ADDRESS "10.0.0.10"
#define CLIENT_ID "T0-R0 video"

int main()
{
  cv::Size bigimg(960,768);
  cv::Size smallimg(320,256);
  cv::Size window(1280,1024);
  cv::Mat win_mat(window, CV_8UC3);
  cv::Mat matPG_small(smallimg, CV_8UC3);
  cv::Mat matbig(bigimg, CV_8UC3);
  cv::Mat Armbuff;
  int set=0;
  std::cout << "select camera /n" << " ";
  scanf("%d", &set);

  // ****************************  ARM camera setup section *******************************

  cv::VideoCapture cap(0);


  const double fps = cap.get(cv::CAP_PROP_FPS);
  const int width  = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  const int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  const int fourcc = cap.get(cv::CAP_PROP_FOURCC );

  // printf("%d", cap.get(CV_CAP_PROP_CONVERT_RGB));
  cv::Mat matARM;


   //if not success, exit program
   if (cap.isOpened() == false)
   {
    std::cout << "Cannot open the ARM camera" << std::endl;
    return -1;
   }
     cv::Mat matARM_small;

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
cv::VideoWriter out("appsrc ! videoconvert ! videoscale ! video/x-raw,format=YUY2,width=1280,height=1024, framerate=30/1 ! jpegenc quality=50 ! rtpjpegpay ! udpsink host=10.0.0.101 port=5000", 1800,0,30, cv::Size(1280,1024), true);

if (out.isOpened()){
	puts("Pipeline Opened");
}

else {
	puts("Pipeline Broken");
}
char key = 0;
int lost =0;

while(key != 'q'){



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

	//cv::imshow("image",image);
        cv::Mat matPG = cv::Mat(rgb.GetRows(), rgb.GetCols(), CV_8UC3, rgb.GetData(),row);
	resize(matPG, matPG_small, smallimg, cv::INTER_CUBIC);

        if(cap.read(Armbuff))
	{
        Armbuff.copyTo(matARM);
        //cap >> matARM;

	}
        //Armbuff = matARM; // get a new frame from camera

  	cv::resize(matARM, matARM_small, smallimg, cv::INTER_CUBIC );


  if (set==1)
  {
   	matbig=matARM;
  }
  else if (set==2)
  {
   	matbig=matPG;
  }
  else
  {
   	matbig=matbig;
  }

  cv::resize(matbig, matbig, bigimg, cv::INTER_CUBIC );

        //cvtColor(matARM,matARM, cv::COLOR_BGR2RGB);



        matbig.copyTo(win_mat(cv::Rect(  0, 0, 960, 768)));
        matPG_small.copyTo(win_mat(cv::Rect(0,760,320,256)));
        matARM_small.copyTo(win_mat(cv::Rect(320,760,320,256)));

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

    return 0;
}
