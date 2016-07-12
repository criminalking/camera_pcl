#include "MoveSenseCamera.h"
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>

// c++
#include <iostream>
#include <stdio.h>

// opencv
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
//#include <imgcodecs/imgcodecs.hpp>
//#include <core/core.hpp>
//#include <highgui/highgui.hpp>
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("points2", 1);

  movesense::CameraMode sel = CAM_STEREO_752X480_LD_30FPS;
  movesense::MoveSenseCamera c(sel);
  if(!(movesense::MS_SUCCESS==c.OpenCamera()))
    {
      std::cout << "Open Camera Failed!" << std::endl;
      std::exit(1);
    }

  int width  = 752;
  int height = 480;
  int len  = width*height*2;

  unsigned char * img_data  = new unsigned char[len];

  pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
  msg->header.frame_id = "map";
  msg->height = height;
  msg->width = width;
  msg->is_dense = false;
  msg->points.resize(msg->width * msg->height);

  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    // for (size_t i = 0; i < msg->points.size (); ++i)
    //   {
    // 	msg->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    // 	msg->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    // 	msg->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    //   }
    
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

	  for(int u = 0; u < height; ++u)
	    for(int v = 0; v < width; ++v)
	      {
	  	int d = disp.at<uchar>(u,v);
	  	if(d<32*4)
	  	  {
	  	    d_real = disp.at<uchar>(u,v)/4;
	  	  }
	  	else
	  	  {
	  	    d_real = (d*2-128)/4.0;
	  	  }
	  	z = b_multipy_f / d_real;
	  	x = (u - cu) * z / f;
	  	y = (v - cv) * z / f;

	  	// save (x,y,z) in pcd
	  	msg->points[u*disp.cols+v].x = x/255.0f;
	  	msg->points[u*disp.cols+v].y = y/255.0f;
	  	msg->points[u*disp.cols+v].z = z/255.0f;
	      }
	  //pcl::io::savePCDFileASCII ("test_pcd.pcd", *msg);
	  
	  cv::imshow("disp",disp);
	  
	  char key = cv::waitKey(10);
	  if(key == 'q')
	    break;
	}
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub.publish (msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
  
}
