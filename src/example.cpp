#include "example.h"

// FIXME: add const
// FIXME: many params have contacts, such as SCALE and HEIGHT

BiCamera::~BiCamera()
{
  delete c;
  delete loop_rate;
  delete[] img_data;
}

void BiCamera::FitPlane(PC::Ptr cloud, PC::Ptr fit_cloud) // need K
{
  // extract a K*K square in the middle of the image 
  const int K = 10;
  int area = 4 * K * K;
  const float kDistanceThreshold = 0.01;
  
  for (int index = 0; index < area; ++index)
      {
	fit_cloud->points[index].x = cloud->points[index].x * 255.0f;
	fit_cloud->points[index].y = cloud->points[index].y * 255.0f;
	fit_cloud->points[index].z = cloud->points[index].z * 255.0f;
      }
  
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
	      
  seg.setInputCloud (fit_cloud);
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
      error += abs(pa * fit_cloud->points[i].x + pb * fit_cloud->points[i].y + pc * fit_cloud->points[i].z + pd);
      ave += abs(pa * fit_cloud->points[i].x + pb * fit_cloud->points[i].y + pc * fit_cloud->points[i].z);
    }
  ave /= area;
  error /= sqrt(pa * pa + pb * pb + pc * pc);
  error /= area;	      
  double distance = -coefficients->values[3]; // the z-value if the plane is horizontal
	      
  if (abs(ave - distance) <= 2 && pc > 0.9999) // insure answer is acceptable
    {
      cout << pa << " " << pb << " " << pc << " " << pd << std::endl;
      cout << "error = " << error << " mm  ";
      cout << "ave = " << ave << " mm  ";
      // show z-axis distance
      cout << "distance = " << distance << " mm" << std::endl;

      // // write in .txt
      // ofstream out;
      // out.open("out.txt", ios::app);
      // out << error << " " << (ave + distance) / 2 << "\n";
      // out.close();
    }
}

void BiCamera::FitLine(PC::Ptr cloud) // exist some errors
{
  const float kDistanceThreshold = 0.01;
  // fitting a line(use point clouds)
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_LINE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (kDistanceThreshold);  
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  // print the coefficients of the line
  for (int i = 0; i < 3; ++i)
    cout << coefficients->values[i] << endl;
}

void BiCamera::Init()
{
  // ros publish
  pub = nh.advertise<PC> ("points_temp", 1);
  pub2 = nh.advertise<PC> ("points_test", 1);

  // set and open camera
  sel = CAM_STEREO_752X480_LD_30FPS;
  c = new movesense::MoveSenseCamera(sel);

  // if use camera
  if(!(movesense::MS_SUCCESS == c->OpenCamera()))
    {
      std::cout << "Open Camera Failed!" << std::endl;
      std::exit(1);
    }

  len  = width * height * 2;
  img_data = new unsigned char[len];
}

void BiCamera::Run()
{
  //Mat left, disp;
  int num = 1;
  char left_name[60];
  char disp_name[60];
  
  // create filter_pointer which points to all template pointclouds
  ProcessTemplate(); // preprocess template
  
  loop_rate = new ros::Rate(4);

  while (nh.ok())
    {
      // if use camera
      c->GetImageData(img_data, len);
      Mat left(height, width, CV_8UC1), disp(height, width, CV_8UC1); // disp is the disparity map
      for(int i = 0 ; i < height; ++i)
      	{
      	  memcpy(left.data + width * i, img_data + (2 * i) * width, width);
      	  memcpy(disp.data + width * i, img_data + (2 * i + 1) * width, width);
      	}

      imshow("left", left);
      imshow("disp", disp);

      char key = waitKey(10);
      if(key == 'q') // quit
  	break;
      // if use camera
      else if (key == 's') // save
      	{
      	  sprintf(left_name, "left_%d.jpg", num);
      	  sprintf(disp_name, "disp_%d.jpg", num);
      	  ++num;
      	  imwrite(left_name, left);
      	  imwrite(disp_name, disp);

	  // compute time
	  clock_t start, finish;
	  double totaltime;
	  start = clock();
  
	  // sprintf(left_name, "img/test/left_%d.jpg", TIM);
	  // sprintf(disp_name, "img/test/disp_%d.jpg", TIM);
	  // left = imread(left_name, 0);
	  // disp = imread(disp_name, 0);
      
	  ProcessTest(disp); // estimate test poses

	  ShowRviz();
            
	  finish = clock();
	  totaltime = (double)(finish - start);
	  cout << "\n run time = " << totaltime / 1000.0 << "ms！" << endl;
      	}
      
      ros::spinOnce ();
      loop_rate->sleep (); // private
    }
}

void BiCamera::ProcessTemplate() 
{
  char left_name[60];
  char disp_name[60];
  Mat left, disp;
  for (int i = 0; i < temp_num; ++i)
    {
      // create temperary point clouds storaging data
      PC::Ptr cloud(new PC);
      cloud->height = height;
      cloud->width = width;
      cloud->is_dense = false;
      cloud->points.resize(cloud->width * cloud->height);

      // read image
      sprintf(left_name, "img/template/left_%d.jpg", i + 1);
      sprintf(disp_name, "img/template/disp_%d.jpg", i + 1);

      left = imread(left_name, 0);
      disp = imread(disp_name, 0);

      //use mid-filter for disp
      medianBlur(disp, disp, MEDIAN);
      
      DepthImageToPc(disp, cloud, 1000, 2000); // depth image convert to point clouds
      
      PC::Ptr cloud_filtered(new PC);
      Filter(cloud, cloud_filtered); // filter point clouds
      GetPeople(cloud_filtered); // get people and remove noises, e.g. celling, ground
      
      PC::Ptr cloud_normalized(new PC);
      Normalize(cloud_filtered, cloud_normalized); // rotate, translate and scale point clouds

      Projection(cloud_normalized); // project to z-plane

      if (i == 1) *cloud_copy0 = *cloud_normalized;
      
      temp_cloud_ptr.push_back(cloud_normalized); // save in vector
    }
  cout << "ProcessTemplate over.\n";
}

void BiCamera::ProcessTest(Mat& disp) 
{
  PC::Ptr cloud(new PC);
  cloud->height = height;
  cloud->width = width;
  cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);

  //use mid-filter for disp
  medianBlur(disp, disp, MEDIAN);

  DepthImageToPc(disp, cloud, 1000, 3000); // depth image convert to point clouds
  
  PC::Ptr cloud_filtered(new PC);
  Filter(cloud, cloud_filtered); // filter point clouds

  GetPeople(cloud_filtered); // get people and remove noises, e.g. celling, ground
  
  PC::Ptr cloud_normalized (new PC);
  Normalize(cloud_filtered, cloud_normalized); // rotate, translate and scale point clouds

  Projection(cloud_normalized); // project to z-plane

  *cloud_copy = *cloud_normalized;
  
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud_normalized, min_pt, max_pt); // get minmum and maximum points in the x-axis(actually y)
  float z_range = max_pt.z - min_pt.z;
  
  // match two point clouds using ICP
  PC::Ptr output(new PC);
  float min = FLT_MAX;
  int min_index = 0;
  for (int i = 0; i < temp_num; ++i)
    {
      // clock_t start, finish;
      // double totaltime;
      // start = clock();
      
      BiCamera::ICP_result result1 = MatchTwoPc(temp_cloud_ptr[i], cloud_normalized, output);
      BiCamera::ICP_result result2 = MatchTwoPc(cloud_normalized, temp_cloud_ptr[i], output);
      
      // finish = clock();
      // totaltime = (double)(finish - start);
      // cout << "\n icp = " << totaltime / 1000.0 << "ms！" << endl;
      
      if (result1.conv == true && result2.conv == true && (result1.score + result2.score) / 2 <= min)
	{
	  min = (result1.score + result2.score) / 2;
	  min_index = i;
	}
    }

  if (min < 10)
    // ROS_WARN("This image is similiar to disp_%d  score: %f\n", min_index + 1, min);
    printf("This image is similiar to disp_%d  score: %f\n", min_index + 1, min);
  else
    printf("Sorry, no similiar images.");
  cout << "ProcessTest over.\n";
}

void BiCamera::DepthImageToPc(Mat& img, PC::Ptr cloud, int down, int up) 
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

  int index = 0;
  // just select neutral part of the image
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

	// storage data to cloud
	if (z > down && z < up) // constraint z-axis
	  {
	    cloud->points[index].x = x / SCALE; // divide SCALE in order to accelerate 
	    cloud->points[index].y = y / SCALE;
	    cloud->points[index].z = z / SCALE;
	    ++index;
	 }
      }
  cloud->points.resize(index); // remove no-data points
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

  //cout << "There are " << cluster_indices.size() << "clusters." << endl;

  // find cluster which has maximum x-range 
  float max_xrange = 0;
  int index = 0;
  int max_index = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    PC::Ptr cloud_cluster (new PC);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
	cloud_cluster->points.push_back (cloud->points[*pit]);
      }

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt); // get minmum and maximum points in the x-axis(actually y)

    // compute z-axis range of this cluster
    float x_range = max_pt.x - min_pt.x;
    if (x_range >= max_xrange)
      {
 	// *cloud = *cloud_cluster; // if here, some strange situations exist.(some strange noises)
     	max_xrange = x_range;
     	max_index = index;
      }
    ++index;

    //cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << endl;
  }

  // assign minimum z-range cluster to cloud
  index = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    if (index == max_index)
      {
	body_z_range.push_back(max_xrange);
  	PC::Ptr cloud_cluster (new PC);
  	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
  	  {
  	    cloud_cluster->points.push_back (cloud->points[*pit]);
  	  }        
  	cloud_cluster->width = cloud_cluster->points.size ();
  	cloud_cluster->height = 1;
  	cloud_cluster->is_dense = true;
  	*cloud = *cloud_cluster;
  	break;
      }
    else ++index;
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
  // first rotate using PCA
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

  // second scale
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
  // third translate
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
  transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
  
  // Print the transformation
  //std::cout << transform_2.matrix() << std::endl;

  // Executing the transformation
  pcl::transformPointCloud (*cloud, *trans, transform_2);
}

void BiCamera::GaussianFilter()
{

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
  msg->height = cloud_copy0->points.size();
  msg->width = 1;
  msg->is_dense = true;
  msg->points.resize(cloud_copy0->points.size()); // necessary

  msg2->header.frame_id = "map";
  msg2->height = cloud_copy->points.size();
  msg2->width = 1;
  msg2->is_dense = true;
  msg2->points.resize(cloud_copy->points.size());
      
  for (size_t i = 0; i < msg->points.size (); ++i)
   {
     msg->points[i].x = cloud_copy0->points[i].x / 255.0 * SCALE;
     msg->points[i].y = cloud_copy0->points[i].y / 255.0 * SCALE;
     msg->points[i].z = cloud_copy0->points[i].z / 255.0 * SCALE;
   }

  // PC::Ptr cloud_transformed (new PC);
  // cloud_transformed->header.frame_id = "map";
  // Eigen::Matrix3d m(1,3);
  // m << 2.0, 0.0, 0.0;
  // Transform(msg, cloud_transformed, M_PI/4, m);

  // PC::Ptr output(new PC);
  // output->header.frame_id = "map";
  // BiCamera::ICP_result a = MatchTwoPc(msg, cloud_transformed, output);
  // cout << a.score << endl;
  // a = MatchTwoPc(cloud_transformed, msg, output);
  // cout << a.score << endl;

  for (size_t i = 0; i < msg2->points.size (); ++i)
    {
      msg2->points[i].x = cloud_copy->points[i].x / 255.0 * SCALE;
      msg2->points[i].y = cloud_copy->points[i].y / 255.0 * SCALE;
      msg2->points[i].z = cloud_copy->points[i].z / 255.0 * SCALE;
    }
      
  pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
  pub.publish (msg);     
  pcl_conversions::toPCL(ros::Time::now(), msg2->header.stamp);
  pub2.publish (msg2);
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  
  BiCamera cam;
  cam.Init();
  cam.Run();
}
