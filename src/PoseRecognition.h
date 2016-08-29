#ifndef POSERECOGNITION_H_
#define POSERECOGNITION_H_

#include <iostream>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <fstream>
#include <time.h>
#include <ros/ros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

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
#include <image_transport/image_transport.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

// my other codes
#include "FaceRecognition.h"
#include "ProcessImage.h"

using namespace std;
using namespace cv;

#define TEMPLATE true
#define TEST false

#define SCALE 20.0 // reduce the value of data in order to accelerate
#define HEIGHT 4.0 * 255 / SCALE // height of a person, for scale
#define MEDIAN 9 // size of kernel (median filtering)

typedef pcl::PointCloud<pcl::PointXYZ> PC;

class BiCamera
{  
public:
  struct ICP_result
  {
    bool conv;
    float score;
  };

  BiCamera(): width(752), height(480), temp_num(9), temp_xy_num(6), cloud_rviz_1(new PC), cloud_rviz_2(new PC) {} 
  ~BiCamera();
  
  void Init(); // initialize
  void Run(); // get a frame and process
  
  bool ProcessTemplate(); // preprocess all template images
  bool ProcessTest(Mat& left, Mat& disp); // only process one test image
  
  void FitPlane(PC::Ptr cloud); // use point clouds to fit a plane
  void DepthImageToPc(Mat& depth_image, PC::Ptr cloud, Rect face); // convert depth-image to point clouds
  bool GetPeople(PC::Ptr cloud); //  get people cluster
  bool Filter(const PC::Ptr cloud, PC::Ptr cloud_filtered); // filter point clouds
  void Normalize(PC::Ptr cloud, PC::Ptr cloud_normalized); // normalize a point cloud, e.g. rotate, translate and scale
  void Projection(PC::Ptr cloud, int flag = 3); // project to plane(flag = 1: x, flag = 2: y, flag = 3: z)
  void Transform(PC::Ptr cloud, PC::Ptr cloud_transformed, float theta, Eigen::Matrix3d m); // transform a point cloud, theta should be radian
  ICP_result MatchTwoPc(PC::Ptr target, PC::Ptr source, PC::Ptr output); // using ICP to match two point clouds(registration)
  void ShowRviz(); // show in rviz
  bool Equal(const pcl::PointXYZ& pt, const pcl::PointXYZ& pt2); // compare two PointXYZ

private:
  Face search_face; // search human face
  Image process_image; // process image(get, save)
  
  // for ros(rviz)
  ros::NodeHandle nh;  
  ros::Publisher pub;
  ros::Publisher pub2;

  // for ros(send and receive image)
  ros::NodeHandle nh_image;
  image_transport::Publisher pub_image;
  ros::Subscriber sub;
  
  ros::Rate *loop_rate;

  // for camera
  int width;
  int height;
  int temp_num; // number of templates
  int temp_xy_num; // number of templates in x-y plane

  vector < PC::Ptr, Eigen::aligned_allocator<PC::Ptr> > temp_cloud_ptr; // store template point clouds
  pcl::PointXYZ face_point; // store midpoint of the face

  // show rviz
  PC::Ptr cloud_rviz_1;
  PC::Ptr cloud_rviz_2;
};

void operator/=(pcl::PointXYZ& pt, double num)
{
  pt.x = pt.x / num;
  pt.y = pt.y / num;
  pt.z = pt.z / num;
}

#endif // POSERECOGNITION_H_
