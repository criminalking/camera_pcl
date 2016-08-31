#include "ShowRviz.h"

void Rviz::ShowRviz()
{
  PC::Ptr msg (new PC);
  PC::Ptr msg2 (new PC);
  PC::Ptr msg3 (new PC);

  msg->header.frame_id = "map"; // necessary
  msg->height = cloud_rviz_computed->points.size();
  msg->width = 1;
  msg->is_dense = true;
  msg->points.resize(cloud_rviz_computed->points.size()); // necessary

  msg2->header.frame_id = "map"; // necessary
  msg2->height = cloud_rviz_right->points.size();
  msg2->width = 1;
  msg2->is_dense = true;
  msg2->points.resize(cloud_rviz_right->points.size()); // necessary

  msg3->header.frame_id = "map"; // necessary
  msg3->height = cloud_rviz_test->points.size();
  msg3->width = 1;
  msg3->is_dense = true;
  msg3->points.resize(cloud_rviz_test->points.size()); // necessary

  for (size_t i = 0; i < msg->points.size(); ++i)
   {
     msg->points[i].x = cloud_rviz_computed->points[i].x / 255.0 * scale;
     msg->points[i].y = cloud_rviz_computed->points[i].y / 255.0 * scale;
     msg->points[i].z = cloud_rviz_computed->points[i].z / 255.0 * scale;
   }

  for (size_t i = 0; i < msg2->points.size (); ++i)
    {
      msg2->points[i].x = cloud_rviz_right->points[i].x / 255.0 * scale;
      msg2->points[i].y = cloud_rviz_right->points[i].y / 255.0 * scale;
      msg2->points[i].z = cloud_rviz_right->points[i].z / 255.0 * scale;
    }

  for (size_t i = 0; i < msg3->points.size (); ++i)
    {
      msg3->points[i].x = cloud_rviz_test->points[i].x / 255.0 * scale;
      msg3->points[i].y = cloud_rviz_test->points[i].y / 255.0 * scale;
      msg3->points[i].z = cloud_rviz_test->points[i].z / 255.0 * scale;
    }
      
  pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
  pub_computed.publish(msg);     
  pcl_conversions::toPCL(ros::Time::now(), msg2->header.stamp);
  pub_right.publish(msg2);
  pcl_conversions::toPCL(ros::Time::now(), msg3->header.stamp);
  pub_test.publish(msg3);
}
