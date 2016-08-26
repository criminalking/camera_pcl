#ifndef PROCESSIMAGE_H_
#define PROCESSIMAGE_H_

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <time.h>

// opencv includes
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "MoveSenseCamera.h"

using namespace std;
using namespace cv;

class Image
{  
 public:
 Image(): width(752), height(480), sel(CAM_STEREO_752X480_LD_30FPS), c(new movesense::MoveSenseCamera(sel)) {} 
  ~Image();
  void GetImage(Mat& left, Mat& disp, int num, bool flag); // get the num-th left and disp
  void SaveImage(Mat& left, Mat& disp, int num, bool flag); // save the num-th left and disp
  void GetImageFromCamera(Mat& left, Mat& disp); // get a image from camera

 private:
  movesense::CameraMode sel;
  movesense::MoveSenseCamera *c;
  int width;
  int height;
  int len;
  unsigned char *img_data;
};

#endif // PROCESSIMAGE_H_
