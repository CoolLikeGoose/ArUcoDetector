cmake_minimum_required(VERSION 3.22.1)

project("aruco")

set(OpenCV_DIR "C:/Programs/sdk/native/jni")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(aruco SHARED src/ArUcoPlugin.cpp)

find_library(log-lib log)

target_link_libraries(aruco
        ${OpenCV_LIBS}
        ${log-lib})
