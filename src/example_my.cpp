// create a subscriber and a publisher for PointCloud2 data

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  std::string topic = nh.resolveName("pcl_output");
  uint32_t queue_size = 1;
  
  // Fill in the cloud data
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 5;
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.points.resize(cloud.width * cloud.height);

  cloud.header.stamp = pcl_conversions::toPCL(ros::Time::now());
  //cloud.header.frame_id = 'map';

  for (size_t i = 0; i < cloud.points.size(); ++i)
    {
      cloud.points[i].x = 3 * i + 1;
      cloud.points[i].y = 3 * i + 2;
      cloud.points[i].z = 3 * i + 3;
    }

  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > (topic, queue_size);

  // convert pcl::PointCloud<pcl::PointXYZ> to sensor_msgs::PointCloud2
  // sensor_msgs::PointCloud2 output;
  // output.header.frame_id = frame;
  // output.header.stamp = pcl_conversions::toPCL(ros::Time::now());//ros::Time::now();
  // pcl::toROSMsg(cloud, output);
  
  // Publish the data
  while(1)
    {
    pub.publish (cloud);
    }
    
  // Spin
  ros::spin ();
}
