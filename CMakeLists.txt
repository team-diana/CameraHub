cmake_minimum_required(VERSION 2.8)

project(camera)

find_package( OpenCV REQUIRED )

add_executable(camera include/FlyCapture2.h T0R0Vision/T0R0Vision.h  T0R0Vision/T0R0Vision.cpp mini.cpp )

target_link_libraries( camera ${OpenCV_LIBS} )
