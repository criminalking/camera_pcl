#include "MoveSenseCamera.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


void TestCase_752X480_LR()
{
  movesense::CameraMode sel = CAM_STEREO_752X480_LR_30FPS;
  movesense::MoveSenseCamera c(sel);
  if(!(movesense::MS_SUCCESS==c.OpenCamera()))
    {
      std::cout << "Open Camera Failed!" << std::endl;
      return;
    }

  int width  = 752;
  int height = 480;
  int len  = width*height*2;

  unsigned char * img_data  = new unsigned char[len];

  while(1)
    {
      c.GetImageData(img_data ,len);
      if(len>0)
	{
	  cv::Mat left(height,width,CV_8UC1),right(height,width,CV_8UC1);
	  for(int i = 0 ; i < height; i++)
	    {
	      memcpy(left.data+width*i,	img_data+(2*i)*width,	width);
	      memcpy(right.data+width*i,	img_data+(2*i+1)*width,	width);
	    }
	  cv::imshow("left",left);
	  cv::imshow("right",right);

	  char key = cv::waitKey(30);
	  if(key == 'q')
	    break;
	}
    }
  c.CloseCamera();
  delete img_data;
}

void TestCase_752X480_LD()
{
  movesense::CameraMode sel = CAM_STEREO_752X480_LD_30FPS;
  movesense::MoveSenseCamera c(sel);
  if(!(movesense::MS_SUCCESS==c.OpenCamera()))
    {
      std::cout << "Open Camera Failed!" << std::endl;
      return;
    }

  int width  = 752;
  int height = 480;
  int len  = width*height*2;

  unsigned char * img_data  = new unsigned char[len];

  // read data in t_P2.xml
  //cv::Mat t_P2;
  //cv::FileStorage fs("t_P2.xml", cv::FileStorage::READ);
  //fs["t_P2"] >> t_P2;

  //save data in pcd file
  pcl::PointCloud<pcl::PointXYZ> cloud;
  //Fill in the cloud data
  cloud.width    = width;
  cloud.height   = height;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  while(1)
    {
      c.GetImageData(img_data ,len);
      if(len > 0)
	{
	  cv::Mat left(height,width,CV_8UC1),disp(height,width,CV_8UC1);
	  for(int i = 0 ; i < height; i++)
	    {
	      memcpy(left.data+width*i,	img_data+(2*i)*width,width);
	      memcpy(disp.data+width*i,img_data+(2*i+1)*width,width);
	    }
	  cv::imshow("left",left);

	  double b_multipy_f = 35981.122607;  //z = b*f/d,b_multipy_f = b*f(only 6 number)
	  double f = 599.065803;
	  double x = 0.00;
	  double y = 0.00;
	  double z = 0.00;
	  double d_real = 0.0;
	  double cu = 369.703644;
	  double cv = 223.365112;

	  for(int k = 0; k < height; ++k)
	    for(int l = 0; l < width; ++l)
	      {
	  	int d = disp.at<uchar>(k,l);
	  	if(d<32*4)
	  	  {
	  	    d_real = disp.at<uchar>(k,l)/4;
	  	  }
	  	else
	  	  {
	  	    d_real = (d*2-128)/4.0;
	  	  }
	  	z = b_multipy_f / d_real;
	  	x = (k - cu) * z / f;
	  	y = (l - cv) * z / f;

	  	// save (x,y,z) in pcd
	  	cloud.points[k*disp.cols+l].x = x/255.0f;
	  	cloud.points[k*disp.cols+l].y = y/255.0f;
	  	cloud.points[k*disp.cols+l].z = z/255.0f;
	      }
	  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
	  
	  cv::imshow("disp",disp);
	  
	  char key = cv::waitKey(10);
	  if(key == 'q')
	    break;
	}
    }
  c.CloseCamera();
  delete img_data;
}

void TestCase_752X480_LRD() // only surport usb3.0 now
{
  movesense::CameraMode sel = CAM_STEREO_752X480_LRD_30FPS;
  movesense::MoveSenseCamera c(sel);
  if(!(movesense::MS_SUCCESS==c.OpenCamera()))
    {
      std::cout << "Open Camera Failed!" << std::endl;
      return;
    }

  int width  = 752;
  int height = 480;
  int len  = width*height*3;

  unsigned char * img_data  = new unsigned char[len];

  while(1)
    {
      c.GetImageData(img_data ,len);
      if(len > 0)
	{
	  cv::Mat left(height,width,CV_8UC1),right(height,width,CV_8UC1),disp(height,width,CV_8UC1);
	  for(int i = 0 ; i < height; i++)
	    {
	      memcpy(left.data+width*i,	img_data+(3*i+0)*width,width);
	      memcpy(right.data+width*i,	img_data+(3*i+1)*width,width);
	      memcpy(disp.data+width*i,	img_data+(3*i+2)*width,width);
	    }
	  cv::imshow("left",left);
	  cv::imshow("right",right);
	  cv::imshow("disp",disp);
	  char key = cv::waitKey(10);
	  if(key == 'q')
	    break;
	}
    }
  c.CloseCamera();
  delete img_data;
}

void TestCase_376X240_LR()
{
  movesense::CameraMode sel = CAM_STEREO_376X240_LR_30FPS;
  movesense::MoveSenseCamera c(sel);
  if(!(movesense::MS_SUCCESS==c.OpenCamera()))
    {
      std::cout << "Open Camera Failed!" << std::endl;
      return;
    }

  int width  = 376;
  int height = 240;
  int len  = width*height*2;

  unsigned char * img_data  = new unsigned char[len];

  while(1)
    {
      c.GetImageData(img_data ,len);
      if(len > 0)
	{
	  cv::Mat left(height,width,CV_8UC1),right(height,width,CV_8UC1);
	  for(int i = 0 ; i < height; i++)
	    {
	      memcpy(left.data+width*i,	img_data+(2*i)*width,width);
	      memcpy(right.data+width*i,img_data+(2*i+1)*width,width);
	    }
	  cv::imshow("left",left);
	  cv::imshow("right",right);
	  char key = cv::waitKey(10);
	  if(key == 'q')
	    break;
	}
    }
  c.CloseCamera();
  delete img_data;
}

void TestCase_376X240_LD()
{
  movesense::CameraMode	sel = CAM_STEREO_376X240_LD_30FPS;
  movesense::MoveSenseCamera c(sel);
  if(!(movesense::MS_SUCCESS==c.OpenCamera()))
    {
      std::cout << "Open Camera Failed!" << std::endl;
      return;
    }

  int width  = 376;
  int height = 240;
  int len  = width*height*2;

  unsigned char * img_data  = new unsigned char[len];

  while(1)
    {
      c.GetImageData(img_data ,len);
      if(len > 0)
	{
	  cv::Mat left(height,width,CV_8UC1),disp(height,width,CV_8UC1);
	  for(int i = 0 ; i < height; i++)
	    {
	      memcpy(left.data+width*i,	img_data+(2*i+0)*width,width);
	      memcpy(disp.data+width*i,	img_data+(2*i+1)*width,width);
	    }
	  cv::imshow("left",left);
	  cv::imshow("disp",disp);
	  char key = cv::waitKey(10);
	  if(key == 'q')
	    break;
	}
    }
  c.CloseCamera();
  delete img_data;
}
void TestCase_376X240_LRD() // only surport usb3.0 now
{
  movesense::CameraMode sel = CAM_STEREO_376X240_LRD_30FPS;
  movesense::MoveSenseCamera c(sel);
  if(!(movesense::MS_SUCCESS==c.OpenCamera()))
    {
      std::cout << "Open Camera Failed!" << std::endl;
      return;
    }

  int width  = 376;
  int height = 240;
  int len  = width*height*3;

  unsigned char * img_data  = new unsigned char[len];

  while(1)
    {
      c.GetImageData(img_data ,len);
      if(len > 0)
	{
	  cv::Mat left(height,width,CV_8UC1),right(height,width,CV_8UC1),disp(height,width,CV_8UC1);
	  for(int i = 0 ; i < height; i++)
	    {
	      memcpy(left.data+width*i,	img_data+(3*i+0)*width,width);
	      memcpy(right.data+width*i,	img_data+(3*i+1)*width,width);
	      memcpy(disp.data+width*i,	img_data+(3*i+2)*width,width);
	    }
	  cv::imshow("left",left);
	  cv::imshow("right",right);
	  cv::imshow("disp",disp);
	  char key = cv::waitKey(10);
	  if(key == 'q')
	    break;
	}
    }
  c.CloseCamera();
  delete img_data;
}



int main()
{
  //TestCase_752X480_LR();    //left right images with full resolution from movesense, default fps is 30
  TestCase_752X480_LD();  //left disparities images with full resolution from movesense, default fps is 30
  //TestCase_752X480_LRD(); //left right disparities images with full resolution from movesense, default fps is 30
  //TestCase_376X240_LR();  //left right images with half resolution from movesense, default fps is 30
  //TestCase_376X240_LD();  //left disparities images with half resolution from movesense, default fps is 30
  //TestCase_376X240_LRD(); //left right disparities images with half resolution from movesense, default fps is 30
  return 0;
}
