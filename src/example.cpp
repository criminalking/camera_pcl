#include <iostream>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include "MoveSenseCamera.h"
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// opencv specific includes
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#define K 50

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

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  cloud->header.frame_id = "map";
  cloud->height = height;
  cloud->width = width;
  cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);

  pcl::PointCloud<pcl::PointXYZ>::Ptr fit_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  fit_cloud->height = K*2;
  fit_cloud->width = K*2;
  fit_cloud->is_dense = true;
  fit_cloud->points.resize(fit_cloud->width * fit_cloud->height);

  ros::Rate loop_rate(4);
  while (nh.ok())
  { 
    c.GetImageData(img_data ,len);
      if(len > 0)
	{
	  cv::Mat left(height,width,CV_8UC1),disp(height,width,CV_8UC1); // disp is the disparity map
	  for(int i = 0 ; i < height; i++)
	    {
	      memcpy(left.data+width*i,	img_data+(2*i)*width,width);
	      memcpy(disp.data+width*i,img_data+(2*i+1)*width,width);
	    }
	  cv::imshow("left",left);
	  cv::imshow("disp",disp);

	  //use mid-filter for disp
	  //cv::medianBlur(disp, disp, 5);

	  double b_multipy_f = 35981.122607;  //z = b*f/d,b_multipy_f = b*f
	  double f = 599.065803;
	  double cu = 369.703644;
	  double cv = 223.365112;
	  
	  double x = 0.00;
	  double y = 0.00;
	  double z = 0.00;
	  double d_real = 0.0;

	  for(int u = 0; u < height; ++u)
	    for(int v = 0; v < width; ++v)
	      {
	  	int d = disp.at<uchar>(u,v);
	  	if(d<32*4)
	  	  {
	  	    d_real = d/4.0;
	  	  }
	  	else
	  	  {
	  	    d_real = (d*2-128)/4.0;
	  	  }
	  	z = b_multipy_f / d_real;
	  	x = (u - cu) * z / f;
	  	y = (v - cv) * z / f;

		//if (z < 3000)
		//{
		// show in rviz
		cloud->points[u*disp.cols+v].x = x/255.0f;
		cloud->points[u*disp.cols+v].y = y/255.0f;
		cloud->points[u*disp.cols+v].z = z/255.0f;
		// }
	      }
	  
	  char key = cv::waitKey(100);
	  if(key == 'q')
	    break;
	  else if (key == 's') // use this frame to test camera
	    {
	      // save the image left and disp
	      cv::imwrite( "left.jpg", left );
	      cv::imwrite( "disp.jpg", disp );
	      // select K*K pixels around the central pixel
	      int index = 0;
	      for (int u = height/2-K; u < height/2+K; ++u)
		for (int v = width/2-K; v < width/2+K; ++v)
		  {
		    fit_cloud->points[index].x = cloud->points[u*disp.cols+v].x * 255.0f;
		    fit_cloud->points[index].y = cloud->points[u*disp.cols+v].y * 255.0f;
		    fit_cloud->points[index].z = cloud->points[u*disp.cols+v].z * 255.0f;
		    index++;
		  }
	      // fitting a plane(use point clouds)
	      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	      // Create the segmentation object
	      pcl::SACSegmentation<pcl::PointXYZ> seg;
	      // Optional
	      seg.setOptimizeCoefficients (true);
	      // Mandatory
	      seg.setModelType (pcl::SACMODEL_PLANE);
	      seg.setMethodType (pcl::SAC_RANSAC);
	      seg.setDistanceThreshold (0.01);
	      
	      seg.setInputCloud (fit_cloud);
	      seg.segment (*inliers, *coefficients);

	      // print the coefficients of the plane
	      double pa = coefficients->values[0];
	      double pb = coefficients->values[1];
	      double pc = coefficients->values[2];
	      double pd = coefficients->values[3];
	      std::cout << pa << " " << pb << " " << pc << " " << pd << std::endl;
	      // compute errors
	      double error = 0.00;
	      for (int i = 0; i < 4*K*K; ++i)
		{
		  error += pow(pa * fit_cloud->points[index].x + pb * fit_cloud->points[index].y + pc * fit_cloud->points[index].z + pd, 2) / 1e+12;
		}
	      error /=  (pa * pa + pb * pb + pc * pc);
	      std::cout << "error = " << error << std::endl;
	      // show z-axis distance
	      double distance = -coefficients->values[3];
	      std::cout << "distance = " << distance << " cm" << std::endl;
	    }
	}
    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    pub.publish (cloud);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
  
}
