#include "ProcessImage.h"

void Image::SaveImage(Mat& left, Mat& disp, int num, bool flag) // flag == true: template, flag == false: test
{
  char left_name[60];
  char disp_name[60];
  if (flag == true)
    {
    sprintf(left_name, "img/template/left_%d.jpg", num);
    sprintf(disp_name, "img/template/disp_%d.jpg", num);
  }
  else
    {
    sprintf(left_name, "img/test/left_%d.jpg", num);
    sprintf(disp_name, "img/test/disp_%d.jpg", num);
  }
  imwrite(left_name, left);
  imwrite(disp_name, disp);
}


void Image::GetImage(Mat& left, Mat& disp, int num, bool flag) // flag == true: template, flag == false: test
{
  char left_name[60];
  char disp_name[60];
  if (flag == true)
    {
    sprintf(left_name, "img/template/left_%d.jpg", num);
    sprintf(disp_name, "img/template/disp_%d.jpg", num);
  }
  else
    {
    sprintf(left_name, "img/test/left_%d.jpg", num);
    sprintf(disp_name, "img/test/disp_%d.jpg", num);
  }
  left = imread(left_name, CV_LOAD_IMAGE_ANYDEPTH);
  disp = imread(disp_name, CV_LOAD_IMAGE_ANYDEPTH);
}
