// create a subscriber and a publisher for PointCloud2 data

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

  pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
  msg->header.frame_id = "map";
  msg->height = msg->width = 1;
  msg->width = 5;
  msg->height = 1;
  msg->is_dense = false;
  msg->points.resize(msg->width * msg->height);

  cv::Mat left(1,1,CV_8UC1);

  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    for (size_t i = 0; i < msg->points.size (); ++i)
      {
	msg->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
	msg->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
	msg->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
      }
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub.publish (msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
  
}
