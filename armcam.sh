gst-launch-1.0 v4l2src  ! videoscale ! videoconvert ! video/x-raw,format=YUY2,width=640,height=480, framerate=30/1 ! jpegenc quality=30 ! rtpjpegpay ! udpsink host=10.0.0.104 port=50216 


