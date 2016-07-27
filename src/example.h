#include <iostream>
#include <stdio.h>
#include <math.h>
#include <fstream>
#include <time.h>
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
#include <pcl/registration/icp.h>
// opencv specific includes
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
// use DBSCAN
#include "dbscan.h"

#define K 10
#define TEMPNUM 6 // define number of templates
typedef pcl::PointCloud<pcl::PointXYZ> PC;

using namespace std;
using namespace cv;
using namespace clustering;

class BiCamera
{
public:
  void ProcessTemplate(); // preprocess all template images
  void ProcessTest(Mat& disp); // only process one test image
  void FitPlane(PC::Ptr cloud, PC::Ptr fit_cloud); // use point clouds to fit a plane 
  void Init(); // initialize
  void Run(); // get a frame and process 
  void DepthImageToPc(Mat& depth_image, PC::Ptr cloud); // convert depth-image to point clouds
  void RemoveNoise(PC::Ptr cloud); // remove noises from environment
  void FilterPc(PC::Ptr cloud, PC::Ptr filter_cloud); // filter point clouds
  float MatchTwoPc(PC::Ptr target, PC::Ptr source, PC::Ptr output); // using ICP to match two point clouds(registration)
  ~BiCamera();

private:
  // for ros
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Rate *loop_rate;

  // for DBSCAN


  // for camera
  movesense::CameraMode sel;
  movesense::MoveSenseCamera *c;
  int width;
  int height;
  int len;
  unsigned char *img_data;
  Mat left, disp, left2, disp2;

  vector < PC::Ptr, Eigen::aligned_allocator<PC::Ptr> > temp_cloud_ptr;

  // start or finish fitting plane
  bool flag;
};
