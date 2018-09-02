cmake_minimum_required (VERSION 2.8)

project (rect)

set(CMAKE_INCLUDE_CURRENT DIR ON)
SET(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH /usr/include)

file(GLOB HEADER_FILE "*.h" "libs/*.h")
file(GLOB SOURCE_FILE "*.cpp" "libs/*.cpp")

add_executable(rect ${HEADER_FILE} ${SOURCE_FILE})
#TARGET_LINK_LIBRARIES(vision pthread ${CMAKE_DL_LIBS})
