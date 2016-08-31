#ifndef SHOWRVIZ_H_
#define SHOWRVIZ_H_

#include <iostream>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <fstream>
#include <time.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PC;

class Rviz
{
 public:
 Rviz(): cloud_rviz_computed(new PC), cloud_rviz_right(new PC), cloud_rviz_test(new PC) {
    // ros publish
    scale = 20.0; // equal SCALE, FIXME: should add config file
    pub_computed = nh.advertise<PC> ("points_computed", 1); 
    pub_right = nh.advertise<PC> ("points_right", 1);
    pub_test = nh.advertise<PC> ("points_test", 1);
  }
  void ShowRviz();
  void SetRvizComputed(PC::Ptr cloud) {*cloud_rviz_computed = *cloud;}
  void SetRvizRight(PC::Ptr cloud) {*cloud_rviz_right = *cloud;}
  void SetRvizTest(PC::Ptr cloud) {*cloud_rviz_test = *cloud;}

 private:
  // for ros(rviz)
  ros::NodeHandle nh;  
  ros::Publisher pub_computed; // show rviz of computed template point clouds
  ros::Publisher pub_right; // show rviz of right template point clouds
  ros::Publisher pub_test; // show rviz of point clouds
  ros::Rate *loop_rate;
  // show rviz
  PC::Ptr cloud_rviz_computed;
  PC::Ptr cloud_rviz_right;
  PC::Ptr cloud_rviz_test;

  double scale;
};


#endif // SHOWRVIZ_H_
