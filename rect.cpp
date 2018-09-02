#include "libs/FlyCapture2.h"
#include "libs/BusManager.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

using namespace FlyCapture2;

#define IP_MQTT_BROKER "10.0.0.126"

#define MQTTCLIENT_QOS2 1

#include "libs/MQTTClient.h"

#define DEFAULT_STACK_SIZE -1

//#include "linux.cpp"

int arrivedcount = 0;

void messageArrived(MQTT::MessageData& md)
{
    MQTT::Message &message = md.message;

	printf("Message %d arrived: qos %d, retained %d, dup %d, packetid %d\n",
		++arrivedcount, message.qos, message.retained, message.dup, message.id);
    printf("Payload %.*s\n", (int)message.payloadlen, (char*)message.payload);
}

int main()
{

    const std::string SERVER_ADDRESS	{ "10.0.0.10" };
    const std::string CLIENT_ID		{ "T0-R0 video" };
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



    //* MQTT connection

    IPStack ipstack = IPStack();
    float version = 0.3;
    const char* topic = "diagnostics";

    printf("MQTT Version is %f\n", version);

    MQTT::Client client = MQTT::Client(ipstack);

    const char* hostname = IP_MQTT_BROKER;
    int port = 1883;
    printf("MQTT: Connecting to %s:%d\n", hostname, port);
    int rc = ipstack.connect(hostname, port);
    if (rc != 0)
        printf("Retrun Code: from TCP connect is %d\n", rc);

    printf("MQTT connecting...\n");
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = (char*)"Camera switch";
    rc = client.connect(data);
    if (rc != 0)
        printf("rc from MQTT connect is %d\n", rc);
    printf("MQTT connected\n");

    rc = client.subscribe(topic, MQTT::QOS2, messageArrived);
    if (rc != 0)
        printf("rc from MQTT subscribe is %d\n", rc);

    //// MQTT ////
    /**/

    while(key != 'q'){      // LOOP



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


    IPStack ipstack = IPStack();
    float version = 0.3;
    const char* topic = "mbed-sample";

    printf("Version is %f\n", version);

    MQTT::Client client = MQTT::Client(ipstack);

    const char* hostname = "iot.eclipse.org";
    int port = 1883;
    printf("Connecting to %s:%d\n", hostname, port);
    int rc = ipstack.connect(hostname, port);
    if (rc != 0)
        printf("rc from TCP connect is %d\n", rc);

    printf("MQTT connecting\n");
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = (char*)"mbed-icraggs";
    rc = client.connect(data);
    if (rc != 0)
        printf("rc from MQTT connect is %d\n", rc);
    printf("MQTT connected\n");

    rc = client.subscribe(topic, MQTT::QOS2, messageArrived);
    if (rc != 0)
        printf("rc from MQTT subscribe is %d\n", rc);

    MQTT::Message message;

    // QoS 0
    char buf[100];
    sprintf(buf, "Hello World!  QoS 0 message from app version %f", version);
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*)buf;
    message.payloadlen = strlen(buf)+1;
    rc = client.publish(topic, message);
    if (rc != 0)
        printf("Error %d from sending QoS 0 message\n", rc);
    else while (arrivedcount == 0)
        client.yield(100);

    rc = client.unsubscribe(topic);
    if (rc != 0)
        printf("rc from unsubscribe was %d\n", rc);

    rc = client.disconnect();
    if (rc != 0)
        printf("rc from disconnect was %d\n", rc);

    ipstack.disconnect();
    return 0;
}
