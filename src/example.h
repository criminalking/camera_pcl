#ifndef EXAMPLE_H_
#define EXAMPLE_H_

#include <iostream>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <fstream>
#include <time.h>
#include <ros/ros.h>

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

// my other codes
#include "MoveSenseCamera.h"
#include "FaceRecognition.h"

#define SCALE 20.0 // reduce the value of data in order to accelerate
#define HEIGHT 5.0 * 255.0 / SCALE // height of a person, for scale
#define MEDIAN 9 // size of kernel (median filtering)

typedef pcl::PointCloud<pcl::PointXYZ> PC;

using namespace std;
using namespace cv;

class BiCamera
{  
public:
  struct ICP_result
  {
    bool conv;
    float score;
  };

 BiCamera(): width(752), height(480), temp_num(5), temp_xy_num(5), cloud_rviz_1(new PC), cloud_rviz_2(new PC) {} 
  ~BiCamera();
  
  void Init(); // initialize
  void Run(); // get a frame and process
  
  void ProcessTemplate(); // preprocess all template images
  void ProcessTest(Mat& left, Mat& disp); // only process one test image
  void FitPlane(PC::Ptr cloud, PC::Ptr fit_cloud); // use point clouds to fit a plane
  Rect FaceRecognition(Mat& img); // recognize human faces
  void DepthImageToPc(Mat& depth_image, PC::Ptr cloud, Rect face); // convert depth-image to point clouds
  void GetPeople(PC::Ptr cloud); //  get people cluster
  void Filter(const PC::Ptr cloud, PC::Ptr cloud_filtered); // filter point clouds
  void Normalize(PC::Ptr cloud, PC::Ptr cloud_normalized); // normalize a point cloud, e.g. rotate, translate and scale
  void Projection(PC::Ptr cloud, int flag = 3); // project to z-plane
  void Transform(PC::Ptr cloud, PC::Ptr cloud_transformed, float theta, Eigen::Matrix3d m); // transform a point cloud, theta should be radian
  ICP_result MatchTwoPc(PC::Ptr target, PC::Ptr source, PC::Ptr output); // using ICP to match two point clouds(registration)
  void ShowRviz(); // show in rviz

private:
  Face search_face; // search human face
  
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
  int temp_num; // number of templates
  int temp_xy_num; // number of templates in x-y plane

  vector < PC::Ptr, Eigen::aligned_allocator<PC::Ptr> > temp_cloud_ptr; // store template point clouds

  pcl::PointXYZ mid_point; // store midpoint of the face

  // show rviz
  PC::Ptr cloud_rviz_1;
  PC::Ptr cloud_rviz_2;
};

bool operator==(const pcl::PointXYZ& pt, const pcl::PointXYZ& pt2)
{
  if (abs(pt2.x - pt.x) < 50 && abs(pt2.y - pt.y) < 50 && abs(pt2.z - pt.z) < 5)
    return true;
  else return false;
}

void operator/=(pcl::PointXYZ& pt, double num)
{
  pt.x = pt.x / num;
  pt.y = pt.y / num;
  pt.z = pt.z / num;
}

#endif // EXAMPLE_H_
