#include "example.h"


BiCamera::~BiCamera()
{
  delete c;
  delete loop_rate;
  delete[] img_data;
}


void BiCamera::FitPlane(PC::Ptr cloud)
{
  double area = cloud->points.size();
  const float kDistanceThreshold = 0.01;
  
  // fitting a plane(use point clouds)
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (kDistanceThreshold);
	      
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  // print the coefficients of the plane
  double pa = coefficients->values[0];
  double pb = coefficients->values[1];
  double pc = coefficients->values[2];
  double pd = coefficients->values[3];
	      
  // compute errors
  double error = 0.00; // sum of the distances (points to plane)
  double ave = 0.00; // the average of the distance
  for (int i = 0; i < area; ++i)
    {
      error += abs(pa * cloud->points[i].x + pb * cloud->points[i].y + pc * cloud->points[i].z + pd);
      ave += abs(pa * cloud->points[i].x + pb * cloud->points[i].y + pc * cloud->points[i].z);
    }
  ave /= area;
  error /= sqrt(pa * pa + pb * pb + pc * pc);
  error /= area;	      
  double distance = -coefficients->values[3]; // the z-value if the plane is horizontal
	      
  //  if (abs(ave - distance) <= 2 && pc > 0.9999) // insure answer is acceptable
	//   {
      cout << pa << " " << pb << " " << pc << " " << pd << std::endl;
      cout << "error = " << error << " mm  ";
      cout << "ave = " << ave << " mm  ";
      // show z-axis distance
      cout << "distance = " << distance << " mm" << std::endl;
      //    }
}


static void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
    {
      cv::imshow("view", cv_bridge::toCvShare(msg, "mono8")->image);
      cv::waitKey(30);
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }
}

void BiCamera::Init()
{
  // ros publish
  pub = nh.advertise<PC> ("points_temp", 1);
  pub2 = nh.advertise<PC> ("points_test", 1);

  
  image_transport::ImageTransport it(nh_image);
  pub_image = it.advertise("camera/image_raw", 1);
  sub_image = it.subscribe("face_det/image_raw", 1, ImageCallback);

  // set and open camera
  sel = CAM_STEREO_752X480_LD_30FPS;
  c = new movesense::MoveSenseCamera(sel);

  // // if use camera
  // if(!(movesense::MS_SUCCESS == c->OpenCamera()))
  //   {
  //     std::cout << "Open Camera Failed!" << std::endl;
  //     std::exit(1);
  //   }

  len  = width * height * 2;
  img_data = new unsigned char[len];
}


void BiCamera::Run()
{
  //Mat left, disp;
  int num = 1;

  loop_rate = new ros::Rate(4);
  
  // create filter_pointer which points to all template pointclouds
  ProcessTemplate(); // preprocess template

  while (nh.ok())
    {
      // if use camera
      Mat left(height, width, CV_8UC1), disp(height, width, CV_8UC1); // disp is the disparity map
      //GetImageFromCamera(left, disp);

      //imshow("left", left);
      //imshow("disp", disp);

      char key = waitKey(10);
      if(key == 'q') // quit
  	break;
      // if use camera
      else if (key == 's') // save
      	{
          // //process_image.SaveImage(left, disp, num, TEST);
      	  // //++num;
	  // //Rect human_face = search_face.Haar(left); // search for human face
	  
  	  // // compute time
  	  // clock_t start, finish;
  	  // double totaltime;
  	  // start = clock();

	  // // read image
	  // if (num > 9) break;
          // process_image.GetImage(left, disp, num, TEST);
	  // ProcessTest(left, disp); // estimate test poses
	  // ++num;
            
  	  // finish = clock();
  	  // totaltime = (double)(finish - start);
  	  // cout << "\n run time = " << totaltime / 1000.0 << "ms！" << endl;
      	}

      ShowRviz();
      
      ros::spinOnce ();
      loop_rate->sleep (); // private
    }
}


void BiCamera::ProcessTemplate() 
{
  Mat left, disp;
  for (int i = 0; i < temp_num; ++i)
    {
      // create temperary point clouds storaging data
      PC::Ptr cloud(new PC);
      cloud->height = height;
      cloud->width = width;
      cloud->is_dense = false;
      cloud->points.resize(cloud->width * cloud->height);

      process_image.GetImage(left, disp, i + 1, TEMPLATE); // get a template image

      medianBlur(disp, disp, MEDIAN); // median filtering

      //Rect human_face = search_face.Haar(left); // search for human face

      // publish image
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", left).toImageMsg();
      // if get the coordinate of the human face, break
      while (nh.ok())
      	{
      	  pub.publish(msg);
      	  ros::spinOnce();
      	  loop_rate->sleep();
      	}
      
      // DepthImageToPc(disp, cloud, human_face); // depth image convert to point clouds
      
      // PC::Ptr cloud_filtered(new PC);
      // Filter(cloud, cloud_filtered); // filter point clouds
      
      // GetPeople(cloud_filtered); // get people cluster

      // pcl::PointXYZ min_pt, max_pt;
      // pcl::getMinMax3D(*cloud_filtered, min_pt, max_pt); // get minmum and maximum points in the z-axis
      // float z_range = mid_point.z - min_pt.z;
      // cout << "z_range: " << z_range << endl;
      
      // PC::Ptr cloud_normalized(new PC);
      // Normalize(cloud_filtered, cloud_normalized); // normalize point clouds

      // if (z_range < 20)
      // 	Projection(cloud_normalized); // project to xy-plane
      // else
      // 	Projection(cloud_normalized, 1); // project to yz-plane

      // if (i == 6) *cloud_rviz_1 = *cloud_normalized;
      // if (i == 7) *cloud_rviz_2 = *cloud_normalized;
      
      // temp_cloud_ptr.push_back(cloud_normalized); // store template point clouds
    }
  cout << "ProcessTemplate over.\n";
}


void BiCamera::ProcessTest(Mat& left, Mat& disp) 
{
  PC::Ptr cloud(new PC);
  cloud->height = height;
  cloud->width = width;
  cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);

  medianBlur(disp, disp, MEDIAN); // median filtering

  Rect human_face = search_face.Haar(left); // search for human face      

  DepthImageToPc(disp, cloud, human_face); // depth image convert to point clouds
  
  PC::Ptr cloud_filtered(new PC);
  Filter(cloud, cloud_filtered); // filter point clouds

  GetPeople(cloud_filtered); // get people cluster
  
  PC::Ptr cloud_normalized (new PC);
  Normalize(cloud_filtered, cloud_normalized); // normalize point clouds

  // compute the distance from face to the frontest part of the human body
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud_normalized, min_pt, max_pt); // get minmum and maximum points in the z-axis
  float z_range = mid_point.z - min_pt.z;
  cout << "z_range: " << z_range << endl;
  
  // match two point clouds using ICP
  PC::Ptr output(new PC);
  float min_value = FLT_MAX;
  int min_index = 0;

  if (z_range < 20) // pose in x-y plane
    {
      Projection(cloud_normalized); // project to xy-plane
      for (int i = 0; i < temp_xy_num; ++i)
  	{      
  	  BiCamera::ICP_result result1 = MatchTwoPc(temp_cloud_ptr[i], cloud_normalized, output);
  	  BiCamera::ICP_result result2 = MatchTwoPc(cloud_normalized, temp_cloud_ptr[i], output);
      
  	  if (result1.conv == true && result2.conv == true && (result1.score + result2.score) / 2 <= min_value)
  	    {
  	      min_value = (result1.score + result2.score) / 2;
  	      min_index = i;
  	    }
  	}
    }
  else // pose in x-y-z space
    {
      Projection(cloud_normalized, 1); // project to yz-plane
      for (int i = temp_xy_num; i < temp_num; ++i)
  	{      
  	  BiCamera::ICP_result result1 = MatchTwoPc(temp_cloud_ptr[i], cloud_normalized, output);
  	  BiCamera::ICP_result result2 = MatchTwoPc(cloud_normalized, temp_cloud_ptr[i], output);
      
  	  if (result1.conv == true && result2.conv == true && (result1.score + result2.score) / 2 <= min_value)
  	    {
  	      min_value = (result1.score + result2.score) / 2;
  	      min_index = i;
  	    }
  	}
    }

  if (min_value < 10)
    printf("This image is similiar to disp_%d  score: %f\n", min_index + 1, min_value);
  else
    printf("Sorry, no similiar images.\n");
  cout << "ProcessTest over.\n";
}


void BiCamera::GetImageFromCamera(Mat& left, Mat& disp)
{
  c->GetImageData(img_data, len);
  for(int i = 0 ; i < height; ++i)
    {
      memcpy(left.data + width * i, img_data + (2 * i) * width, width);
      memcpy(disp.data + width * i, img_data + (2 * i + 1) * width, width);
    }
}


void BiCamera::DepthImageToPc(Mat& img, PC::Ptr cloud, Rect face) 
{
  // camera params
  const double kBMultipyF = 35981.122607;  // kBMultipyF = b*f
  const double kF = 599.065803;
  const double kCu = 369.703644;
  const double kCv = 223.365112;
	
  double x = 0.00;
  double y = 0.00;
  double z = 0.00;
  double d_real = 0.0;

  // get the midpoint of the human face
  int u = face.y + face.height / 2;
  int v = face.x + face.width / 2;
  int d = img.at<uchar>(u, v) + 1.0; // avoid zero
  if(d < 128)
    d_real = d / 4.0;
  else
    d_real = (d * 2 - 128) / 4.0;
  mid_point.z = kBMultipyF / d_real;
  mid_point.x = (u - kCu) * mid_point.z / kF;
  mid_point.y = (v - kCv) * mid_point.z / kF;
  cout << "mid_point" << mid_point << endl;
  
  // convert depth image to point cloud (z-value has constraint)
  int index = 0;
  for (int u = 0; u < height; ++u)
    for (int v = 0; v < width; ++v)
      {
  	int d = img.at<uchar>(u,v) + 1.0; // avoid zero
  	if(d < 128)
  	  d_real = d / 4.0;
  	else
  	  d_real = (d * 2 - 128) / 4.0;

  	// compute (x, y, z)
  	z = kBMultipyF / d_real;
  	x = (u - kCu) * z / kF;
  	y = (v - kCv) * z / kF;

  	// store data to cloud
	if (z > 1000 && z < mid_point.z + 200) // constraint z-value
	  {
  	    cloud->points[index].x = x / SCALE; // divide SCALE in order to accelerate 
  	    cloud->points[index].y = y / SCALE;
  	    cloud->points[index].z = z / SCALE;
  	    ++index;
	  }
      }
  cloud->points.resize(index); // remove no-data points 

  mid_point /= SCALE;
  cout << "mid_point / SCALE: " << mid_point << endl;
}


void BiCamera::GetPeople(PC::Ptr cloud)
{
  const float kClusterTolerance = 5.0;
  const int kMinClusterSize = 1000;
  const int kMaxClusterSize = 20000;
  
  // search for connected domain(cluster) to remove ground or wall
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(kClusterTolerance); // tolerant distance
  ec.setMinClusterSize(kMinClusterSize); // minimum cluster size
  ec.setMaxClusterSize(kMaxClusterSize); // maximum cluster size
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  bool flag;
  int num = 0;

  if (cluster_indices.size() == 0)
    cout << "No people cluster.\n";
  else
    { 
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  	{
  	  PC::Ptr cloud_cluster (new PC);
  	  flag = false;
  	  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
  	    {
  	      cloud_cluster->points.push_back(cloud->points[*pit]);
  	      if (Equal(cloud->points[*pit], mid_point) == true) // the mid_point maybe be filtered
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
      if (flag == false) cout << "No people cluster.\n"; // midpoint is not in valid clusters
    }
}


void BiCamera::Filter(const PC::Ptr cloud, PC::Ptr cloud_filtered)
{
  const float kFilterSize = 1.2;
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(kFilterSize, kFilterSize, kFilterSize);
  vg.filter(*cloud_filtered);
  cout << cloud->points.size() << "   " << cloud_filtered->points.size() << endl;
}


void BiCamera::Normalize(PC::Ptr cloud, PC::Ptr cloud_normalized)
{
  const float kHeadScale = 9;
  // first: rotate using PCA
  // pcl::PCA<pcl::PointXYZ> pca(*cloud);
  // Eigen::Matrix3f eigen_vector = pca.getEigenVectors();
  // cout << endl << eigen_vector << endl;
  // float tan_theta = eigen_vector(1, 0) / eigen_vector(0, 0);
  // float theta = atan (tan_theta); // degree
  // cout << "theta " << theta * 180 / M_PI << endl;
  
  Eigen::Matrix3d m(1, 3);
  m << 0.0, 0.0, 0.0;
  PC::Ptr cloud_transformed (new PC);
  Transform(cloud, cloud_transformed, 0, m);

  // second: scale
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud_transformed, min_pt, max_pt); // get minmum and maximum points in the x-axis(actually y)
  float x_range = max_pt.x - min_pt.x; // range of x-axis
  float x_middle = x_range / 2 + min_pt.x;
  float scale = HEIGHT / x_range;
  float y_offset = 0.0;
  int index = 0;
  
  // all points should multiple scale
  for (int i = 0; i < cloud->points.size(); ++i)
    {
      if ((max_pt.x - cloud_transformed->points[i].x) < x_range / kHeadScale) // head
  	{
  	  y_offset += cloud_transformed->points[i].y;
  	  ++index;
  	}
      cloud_transformed->points[i].x *= scale;
      cloud_transformed->points[i].y *= scale;
      cloud_transformed->points[i].z *= scale;
    }

  y_offset = y_offset * scale / index;
  
  // third: translate
  m << -x_middle * scale, -y_offset, 0.0;
  Transform(cloud_transformed, cloud_normalized, 0, m);
}


void BiCamera::Projection(PC::Ptr cloud, int flag)
{
  if (flag == 1)
    for (int i = 0; i < cloud->points.size(); ++i)
      cloud->points[i].x = 0;
  else if (flag == 2)
    for (int i = 0; i < cloud->points.size(); ++i)
      cloud->points[i].y = 0;
  else if (flag == 3)
    for (int i = 0; i < cloud->points.size(); ++i)
      cloud->points[i].z = 0;   
}


void BiCamera::Transform(PC::Ptr cloud, PC::Ptr trans, float theta, Eigen::Matrix3d m)
{
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  transform_2.translation() << m(0,0), m(0,1), m(0,2);
  // theta radians arround Z axis
  transform_2.rotate (Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
  // Executing the transformation
  pcl::transformPointCloud (*cloud, *trans, transform_2);
}


BiCamera::ICP_result BiCamera::MatchTwoPc(PC::Ptr target, PC::Ptr source, PC::Ptr output) // ICP
{
  const float kIcpDistance = 0.05;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;  
  icp.setMaxCorrespondenceDistance(kIcpDistance); // this param is very important!!!  
  icp.setTransformationEpsilon(1e-10); // if difference between two transformation matrix smaller than threshold, converge
  icp.setEuclideanFitnessEpsilon(0.01); // if sum of MSE smaller than threshold, converge
  icp.setMaximumIterations(100); // if iteration smaller than threshold, converge
  
  icp.setInputSource(source);  
  icp.setInputTarget(target);  
  icp.align(*output);  
  cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;  
  
  output->resize(target->size() + output->size());  
  for (int i = 0; i < target->size(); ++i)  
    output->push_back(target->points[i]); // merge two points 
  //cout << icp.getFinalTransformation() << endl;

  BiCamera::ICP_result result;
  result.conv = icp.hasConverged();
  result.score = icp.getFitnessScore();
  return result;
}


void BiCamera::ShowRviz()
{
  PC::Ptr msg (new PC);
  PC::Ptr msg2 (new PC);

  msg->header.frame_id = "map";
  msg->height = cloud_rviz_1->points.size();
  msg->width = 1;
  msg->is_dense = true;
  msg->points.resize(cloud_rviz_1->points.size()); // necessary

  msg2->header.frame_id = "map";
  msg2->height = cloud_rviz_2->points.size();
  msg2->width = 1;
  msg2->is_dense = true;
  msg2->points.resize(cloud_rviz_2->points.size());
      
  for (size_t i = 0; i < msg->points.size (); ++i)
   {
     msg->points[i].x = cloud_rviz_1->points[i].x / 255.0 * SCALE;
     msg->points[i].y = cloud_rviz_1->points[i].y / 255.0 * SCALE;
     msg->points[i].z = cloud_rviz_1->points[i].z / 255.0 * SCALE;
   }

  for (size_t i = 0; i < msg2->points.size (); ++i)
    {
      msg2->points[i].x = cloud_rviz_2->points[i].x / 255.0 * SCALE;
      msg2->points[i].y = cloud_rviz_2->points[i].y / 255.0 * SCALE;
      msg2->points[i].z = cloud_rviz_2->points[i].z / 255.0 * SCALE;
    }
      
  pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
  pub.publish (msg);     
  pcl_conversions::toPCL(ros::Time::now(), msg2->header.stamp);
  pub2.publish (msg2);
}


bool BiCamera::Equal(const pcl::PointXYZ& pt, const pcl::PointXYZ& pt2)
{
  if (abs(pt2.x - pt.x) < 5 && abs(pt2.y - pt.y) < 5 && abs(pt2.z - pt.z) < 5)
    return true;
  else return false;
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  
  BiCamera cam;
  cam.Init();
  cam.Run();
}
