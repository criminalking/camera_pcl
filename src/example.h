#include <iostream>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <fstream>
#include <time.h>
#include <ros/ros.h>
#include "MoveSenseCamera.h"
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>

#include <pcl/registration/icp.h>
// opencv specific includes
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
// use DBSCAN
//#include "dbscan.h"

#define K 10
#define TEMPNUM 6 // define number of templates
#define SCALE 20.0 // reduce the value of data in order to accelerate
#define HEIGHT 5.0 // height of a person, for scale 
typedef pcl::PointCloud<pcl::PointXYZ> PC;

using namespace std;
using namespace cv;
//using namespace clustering;

// just for test
#define RVIZ 1 // rviz show the RVIZ-th template
#define TIM 7 // number of test image

struct ICP_result
{
  bool conv;
  float score;
};

class BiCamera
{
public:
  void Init(); // initialize
  void Run(); // get a frame and process
  
  void ProcessTemplate(); // preprocess all template images
  void ProcessTest(Mat& disp); // only process one test image
  void FitPlane(PC::Ptr cloud, PC::Ptr fit_cloud); // use point clouds to fit a plane 
  void DepthImageToPc(Mat& depth_image, PC::Ptr cloud); // convert depth-image to point clouds
  void RemoveNoise(PC::Ptr cloud); // remove noises, e.g. celling, ground
  void FilterPc(PC::Ptr cloud, PC::Ptr cloud_filtered); // filter point clouds
  ICP_result MatchTwoPc(PC::Ptr target, PC::Ptr source, PC::Ptr output); // using ICP to match two point clouds(registration)
  void ShowRviz(); // show in rviz
  void Transform(PC::Ptr cloud, PC::Ptr cloud_transformed, float theta, Eigen::Matrix3d m); // transform a point clouds, theta should be radian
  void Normalize(PC::Ptr cloud, PC::Ptr cloud_normalized); // normalize a point clouds, e.g. rotate and scale
  void FitLine(PC::Ptr cloud);
  
  ~BiCamera();

private:
  // for ros
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Publisher pub2;
  ros::Rate *loop_rate;

  // for camera
  movesense::CameraMode sel;
  movesense::MoveSenseCamera *c;
  int width;
  int height;
  int len;
  unsigned char *img_data;
  //Mat left, disp;

  vector < PC::Ptr, Eigen::aligned_allocator<PC::Ptr> > temp_cloud_ptr;

  // start or finish fitting plane
  bool flag;
};
