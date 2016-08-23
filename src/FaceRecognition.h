#ifndef FACERECOGNITION_H_
#define FACERECOGNITION_H_

#include <iostream>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <fstream>
#include <time.h>

// opencv includes
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

class Face
{
 public:
  Face(): face_num(1) {} 
  Rect Haar(Mat& img); // method 1: haar
  

 private:
  // store face images
  int face_num;
};

#endif // FACERECOGNITION_H_
