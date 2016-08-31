#include "GetPeople.h"


People::~People()
{
  delete loop_rate;
}


void People::Segmentation(Mat& img, PC::Ptr cloud, Rect face, pcl::PointXYZ& face_mid_point)
{
  const double kBMultipyF = 35981.122607;  // kBMultipyF = b*f
  const double kF = 599.065803;
  const double kCu = 369.703644;
  const double kCv = 223.365112;

  double d_real = 0.0;
  
  // get the midpoint of the human face
  int u = face.y + face.height / 2;
  int v = face.x + face.width / 2;
  int d = img.at<uchar>(u, v) + 1.0; // avoid zero
  if(d < 128)
    d_real = d / 4.0;
  else
    d_real = (d * 2 - 128) / 4.0;
  face_mid_point.z = kBMultipyF / d_real;
  face_mid_point.x = (u - kCu) * face_mid_point.z / kF;
  face_mid_point.y = (v - kCv) * face_mid_point.z / kF;
  cout << "face_mid_point" << face_mid_point << endl;

  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
  range_cond->addComparison(
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, face_mid_point.z + 250))
  );
  range_cond->addComparison(
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 1000))
  );
  pcl::ConditionalRemoval<pcl::PointXYZ> condrem(range_cond);
  condrem.setInputCloud(cloud);            
  condrem.filter(*cloud);        

  for (size_t i = 0; i < cloud->points.size(); ++i)
    {
      cloud->points[i].x /= SCALE; // divide SCALE in order to accelerate 
      cloud->points[i].y /= SCALE;
      cloud->points[i].z /= SCALE;
    }

  face_mid_point.x /= SCALE;
  face_mid_point.y /= SCALE;
  face_mid_point.z /= SCALE;
}


bool People::Filter(const PC::Ptr cloud, PC::Ptr cloud_filtered)
{
  const float kFilterSize = 1.2;
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(kFilterSize, kFilterSize, kFilterSize);
  vg.filter(*cloud_filtered);
  cout << "Filter: " << cloud->points.size() << " " << cloud_filtered->points.size() << endl;
  if (cloud_filtered->points.size() > 10000 || cloud_filtered->points.size() < 1000)
    {
      ROS_WARN("Filtering failed.\n");
      return false;
    }
  else return true;
}


bool People::Equal(const pcl::PointXYZ& pt, const pcl::PointXYZ& pt2)
{
  if (abs(pt2.x - pt.x) < 5 && abs(pt2.y - pt.y) < 5 && abs(pt2.z - pt.z) < 5)
    return true;
  else return false;
}


bool People::TrackFace(Mat& left, Mat& disp, PC::Ptr cloud, pcl::PointXYZ& face_mid_point)
{
  loop_rate = new ros::Rate(4);
  // using Dlib-method to search for human face
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", left).toImageMsg();
  flag_sub = false;
  while (flag_sub == false)
    {
      pub_image.publish(msg); // send image to human_face_detection_package
      loop_rate->sleep();
      ros::spinOnce(); // Attention: this line should be set at last, otherwise some errors exist
    }
  Rect human_face = search_face.Dlib(Arr);
  if (human_face == cv::Rect(0, 0, 0, 0)) return false; // no human face

  // constraint point clouds in z_range and compress
  Segmentation(disp, cloud, human_face, face_mid_point);
  
  // filter point clouds
  PC::Ptr cloud_filtered(new PC);
  if (Filter(cloud, cloud_filtered) == false) 
    return false; 

  const float kClusterTolerance = 5.0;
  const int kMinClusterSize = 1000;
  const int kMaxClusterSize = 20000;
  
  // search for connected domain(cluster) to remove ground or wall
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(kClusterTolerance); // tolerant distance
  ec.setMinClusterSize(kMinClusterSize); // minimum cluster size
  ec.setMaxClusterSize(kMaxClusterSize); // maximum cluster size
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  bool flag;
  int num = 0;

  if (cluster_indices.size() == 0)
    {
      ROS_WARN("No people cluster.\n");
      return false;
    }
  else
    { 
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  	{
  	  PC::Ptr cloud_cluster (new PC);
  	  flag = false;
  	  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
  	    {
  	      cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
  	      if (Equal(cloud_filtered->points[*pit], face_mid_point) == true) // the face_mid_point maybe be filtered
		{
		  flag = true; // this cluster contains the midpoint
		  ++num;
		}
  	    }
  	  if (flag == true && num > 50)
  	    {
  	      cloud_cluster->width = cloud_cluster->points.size();
  	      cloud_cluster->height = 1;
  	      cloud_cluster->is_dense = true;
  	      *cloud = *cloud_cluster;
	      break;
  	    }
  	}
      if (flag == false)
	{
	  ROS_WARN("No people cluster.\n"); // midpoint is not in valid clusters
	  return false;
	}
    }
  return true;
}


void People::TrackBody(PC::Ptr cloud) // need to be improved
{

}
