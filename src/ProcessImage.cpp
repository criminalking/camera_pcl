#include "ProcessImage.h"

Image::~Image()
{
  delete c;
  delete[] img_data;
}

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


void Image::GetImageFromCamera(Mat& left, Mat& disp)
{
  if(!(movesense::MS_SUCCESS == c->OpenCamera()))
    {
      std::cout << "Open Camera Failed!" << std::endl;
      std::exit(1);
    }

  len  = width * height * 2;
  img_data = new unsigned char[len];
  
  c->GetImageData(img_data, len);
  for(int i = 0 ; i < height; ++i)
    {
      memcpy(left.data + width * i, img_data + (2 * i) * width, width);
      memcpy(disp.data + width * i, img_data + (2 * i + 1) * width, width);
    }
}
