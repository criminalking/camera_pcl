#include <iostream>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <fstream>
#include <time.h>
#include <ros/ros.h>
#include "MoveSenseCamera.h"
// PCL includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
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

// opencv includes
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#define SCALE 20.0 // reduce the value of data in order to accelerate
#define HEIGHT 5.0 * 255.0 / SCALE // height of a person, for scale

typedef pcl::PointCloud<pcl::PointXYZ> PC;

using namespace std;
using namespace cv;

// just for test
#define RVIZ 2 // rviz show the RVIZ-th template
#define TIM 9 // number of test image

class BiCamera
{  
public:
  struct ICP_result
  {
    bool conv;
    float score;
  };

  BiCamera(): width(752), height(480), temp_num(6) {} 
  ~BiCamera();
  
  void Init(); // initialize
  void Run(); // get a frame and process
  
  void ProcessTemplate(); // preprocess all template images
  void ProcessTest(Mat& disp); // only process one test image
  void FitPlane(PC::Ptr cloud, PC::Ptr fit_cloud); // use point clouds to fit a plane 
  void DepthImageToPc(Mat& depth_image, PC::Ptr cloud); // convert depth-image to point clouds
  void GetPeople(PC::Ptr cloud); // get people and remove noises, e.g. celling, ground
  void Filter(PC::Ptr cloud, PC::Ptr cloud_filtered); // filter point clouds
  ICP_result MatchTwoPc(PC::Ptr target, PC::Ptr source, PC::Ptr output); // using ICP to match two point clouds(registration)
  void ShowRviz(); // show in rviz
  void Transform(PC::Ptr cloud, PC::Ptr cloud_transformed, float theta, Eigen::Matrix3d m); // transform a point cloud, theta should be radian
  void Normalize(PC::Ptr cloud, PC::Ptr cloud_normalized); // normalize a point cloud, e.g. rotate, translate and scale
  void FitLine(PC::Ptr cloud); // fit a line of point clouds
  void Projection(PC::Ptr cloud); // project to z-plane

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
  int temp_num; // number of template
  //Mat left, disp;

  vector < PC::Ptr, Eigen::aligned_allocator<PC::Ptr> > temp_cloud_ptr;

};
