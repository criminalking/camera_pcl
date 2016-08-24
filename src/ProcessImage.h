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

using namespace std;
using namespace cv;

class Image
{  
 public:
  void GetImage(Mat& left, Mat& disp, int num, bool flag); // get the num-th left and disp
  void SaveImage(Mat& left, Mat& disp, int num, bool flag); // save the num-th left and disp
};

#endif // PROCESSIMAGE_H_
