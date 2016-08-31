#ifndef POSERECOGNITION_H_
#define POSERECOGNITION_H_

#include <iostream>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <fstream>
#include <time.h>
#include <ros/ros.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

// opencv includes
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

// my other codes
#include "ProcessImage.h"
#include "GetPeople.h"

using namespace std;
using namespace cv;

#define TEMPLATE true
#define TEST false
#define CAMERA true
#define IMAGE false

#define SCALE 20.0 // reduce the value of data in order to accelerate
#define HEIGHT 4.0 * 255 / SCALE // height of a person, for scale
#define ZRANGE 20 // if z_range < ZRANGE, this pose is only in xy-plane

typedef pcl::PointCloud<pcl::PointXYZ> PC;

// right answer of 68 test images
int answer[68] = {8, 10, 1, 2, 3, 4, 6, 5, 0, 10,
                  8, 9, 7, 1, 2, 4, 3, 6, 5, 9,
                  7, 1, 2, 4, 3, 6, 5, 8, 10, 9,
                  7, 1, 2, 4, 3, 6, 5, 10, 8, 9,
                  0, 1, 2, 3, 4, 6, 5, 8, 10, 9,
                  7, 1, 4, 3, 2, 5, 6, 0, 10, 8,
                  9, 7, 1, 2, 4, 3, 6, 5};         

class BiCamera
{  
public:
  struct ICP_result
  {
    bool conv;
    float score;
  };

 BiCamera(): width(752), height(480), temp_num(10), temp_xy_num(7), cloud_rviz_computed(new PC), cloud_rviz_right(new PC), cloud_rviz_test(new PC) {} 
  ~BiCamera();
  
  void Init(); // initialize
  void Run(bool flag); // get a frame and process, flag == true: use camera, flag == false: use test images
  
  bool ProcessTemplate(); // preprocess all template images
  bool ProcessTest(Mat& left, Mat& disp, int& ans); // only process one test image

private:
  void FitPlane(PC::Ptr cloud); // use point clouds to fit a plane
  void DepthImageToPc(Mat& depth_image, PC::Ptr cloud); // convert depth-image to point clouds
  void Normalize(PC::Ptr cloud, PC::Ptr cloud_normalized); // normalize a point cloud, e.g. rotate, translate and scale
  void Projection(PC::Ptr cloud, int flag = 3); // project to plane(flag = 1: x, flag = 2: y, flag = 3: z)
  void Transform(PC::Ptr cloud, PC::Ptr cloud_transformed, float theta, Eigen::Matrix3d m); // transform a point cloud, theta should be radian
  ICP_result MatchTwoPc(PC::Ptr target, PC::Ptr source, PC::Ptr output); // using ICP to match two point clouds(registration)
  bool PoseMatching(float z_range, PC::Ptr cloud_normalized, int& ans); // search for the similarest pose 
  void ShowRviz(); // show in rviz
  
  Image process_image; // process image(get, save)
  People get_people; // get people cluster
  
  // for ros(rviz)
  ros::NodeHandle nh;  
  ros::Publisher pub_computed; // show rviz of computed template point clouds
  ros::Publisher pub_right; // show rviz of right template point clouds
  ros::Publisher pub_test; // show rviz of point clouds
  ros::Rate *loop_rate;

  int width; // width of image
  int height; // height of image
  int temp_num; // number of templates
  int temp_xy_num; // number of templates in x-y plane

  vector < PC::Ptr, Eigen::aligned_allocator<PC::Ptr> > temp_cloud_ptr; // store template point clouds
  pcl::PointXYZ face_mid_point; // store midpoint of the face

  // show rviz
  PC::Ptr cloud_rviz_computed;
  PC::Ptr cloud_rviz_right;
  PC::Ptr cloud_rviz_test;
};

#endif // POSERECOGNITION_H_
