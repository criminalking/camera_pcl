#include <iostream>
#include <stdio.h>
#include <math.h>
#include <fstream>
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

#define K 20
typedef pcl::PointCloud<pcl::PointXYZ> PC;

class BiCamera
{
public:
  void FitPlane(PC::Ptr cloud, PC::Ptr fit_cloud); // use point clouds to fit a plane 
  void Init(PC::Ptr cloud, PC::Ptr fit_cloud); // initialize
  void Run(PC::Ptr cloud, PC::Ptr fit_cloud); // get a frame and process 
  void DepthImageToPc(cv::Mat disp, PC::Ptr cloud); // convert depth-image to point clouds
  ~BiCamera();

private:
  // for ros
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Rate *loop_rate;

  // for camera
  movesense::CameraMode sel;
  movesense::MoveSenseCamera *c;
  int width;
  int height;
  int len;
  unsigned char * img_data;

  // start or finish fitting plane
  bool flag;
};
