#include "FlyCapture2.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace FlyCapture2;

int main()
{
    Error error;
    Camera camera;
    CameraInfo camInfo;

    // Connect the camera
    error = camera.Connect( 0 );
    if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to connect to camera" << std::endl;     
        //return false;
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
cv::VideoWriter out("appsrc ! videoscale  ! videoconvert ! video/x-raw,format=YUY2,width=800,height=600, framerate=30/1 ! jpegenc quality=30  ! rtpjpegpay ! udpsink host=10.0.0.104 port=50215", 1800,0,30, cv::Size(1280,1024), true);

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
		std::cout << "network loss frame" << std::endl;
                lost++;
		continue;
	}
//	printf("Frames lost: %d \n", lost);
	Image rgb;
	raw.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgb );
	
	unsigned int row = (double)rgb.GetReceivedDataSize()/(double)rgb.GetRows();
	
	cv::Mat image = cv::Mat(rgb.GetRows(), rgb.GetCols(), CV_8UC3, rgb.GetData(),row);
	
//	cv::imshow("image",image);
        
	out.write(image);
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

