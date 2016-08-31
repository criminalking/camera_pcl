#ifndef GETPEOPLE_H_
#define GETPEOPLE_H_

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

// PCL includes
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

// opencv includes
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

// my other codes
#include "FaceRecognition.h"

#define SCALE 20.0 // reduce the value of data in order to accelerate

using namespace std;
using namespace cv;
typedef pcl::PointCloud<pcl::PointXYZ> PC;

bool flag_sub;
int Arr[200];

// Once get the message from package face_detection, this function is called
static void ChatterCallback(const std_msgs::Int32MultiArray::ConstPtr& myMsg)
{
  int i = 0;
  //the data is being stored inside an array
  for(std::vector<int>::const_iterator it = myMsg->data.begin(); it != myMsg->data.end(); ++it)
  {
    Arr[i] = *it;
    i++;
  }
  flag_sub = true;
}

class People
{
 public:
 People(): width(752), height(480) {
    image_transport::ImageTransport it(nh_image);
    pub_image = it.advertise("camera/image_raw", 1);
    sub = nh_image.subscribe("faceCoord", 1000, ChatterCallback);
    flag_sub = false; // a trigger
  }
  ~People();
  bool TrackFace(Mat& left, Mat& disp, PC::Ptr cloud, pcl::PointXYZ& face_mid_point); // method 1: track face in order to get people
  void TrackBody(PC::Ptr cloud);// method 2: track human-body directly(need to be improved)

 private:
  bool Filter(const PC::Ptr cloud, PC::Ptr cloud_filtered); // filter point clouds
  bool Equal(const pcl::PointXYZ& pt, const pcl::PointXYZ& pt2); // compare two PointXYZ
  void Segmentation(Mat& depth_image, PC::Ptr cloud, Rect face, pcl::PointXYZ& face_mid_point); // constraint point clouds in a z_range and compress point clouds

  Face search_face; // search human face

  int width; // width of image
  int height; // height of image

  // for ros(send and receive image)
  ros::NodeHandle nh_image;
  image_transport::Publisher pub_image;
  ros::Subscriber sub;
  ros::Rate *loop_rate;
};

#endif // GETPEOPLE_H_
