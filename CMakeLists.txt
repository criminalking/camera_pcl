cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# add address of dlib 
include(/home/criminalking/Softwares/dlib/dlib/cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(CMAKE_C_FLAGS "-std=gnu99 -pedantic -fPIC -pedantic -fopenmp -mtune=corei7-avx")
SET(CMAKE_CXX_FLAGS "-std=c++0x -Wall -fPIC -pedantic -fopenmp -mtune=corei7-avx")

#for PCL
find_package(PCL 1.8.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#for openCV
find_package(cv_bridge)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#libraries
add_library(CtrlInf src/CtrlInf.cpp)
add_library(CameraCtrl src/CameraCtrl.cpp)
target_link_libraries( CameraCtrl CtrlInf)

add_library(VL4IF src/VL4IF.cpp )
target_link_libraries( VL4IF udev)

add_library(Camera src/Camera.cpp)
target_link_libraries( Camera VL4IF)

add_library(MoveSenseCamera src/MoveSenseCamera.cpp)
target_link_libraries( MoveSenseCamera Camera)

add_library(FaceRecognition src/FaceRecognition.cpp)
add_library(ProcessImage src/ProcessImage.cpp)

rosbuild_add_executable(example src/example.cpp)
target_link_libraries(  example  VL4IF CameraCtrl Camera MoveSenseCamera FaceRecognition ProcessImage ${OpenCV_LIBS} dlib)
INCLUDE_DIRECTORIES(  ${OPENCV_INCLUDE_DIR} )

#for boost
find_package(Boost 1.58.0 REQUIRED COMPONENTS system)
#message(${Boost_LIBRARY_DIRS})
#message(${Boost_LIBRARIES})
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
target_link_libraries(example ${Boost_LIBRARIES})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#rosbuild_add_library(${PROJECT_NAME} src/Camera.cpp)
target_link_libraries(example ${OpenCV_LIBRARIES})
target_link_libraries(example ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
