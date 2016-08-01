#include "example.h"

BiCamera::~BiCamera()
{
  delete c;
  delete loop_rate;
  delete[] img_data;
}

void BiCamera::FitPlane(PC::Ptr cloud, PC::Ptr fit_cloud) // need K
{
  // select K*K pixels around the central pixel  
  for (int index = 0; index < 4 * K * K; ++index)
      {
	fit_cloud->points[index].x = cloud->points[index].x * 255.0f;
	fit_cloud->points[index].y = cloud->points[index].y * 255.0f;
	fit_cloud->points[index].z = cloud->points[index].z * 255.0f;
      }

  //clock_t start, finish;
  //double totaltime;
  //start = clock();
  
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
  seg.setDistanceThreshold (0.01);
	      
  seg.setInputCloud (fit_cloud);
  seg.segment (*inliers, *coefficients);

  //finish = clock();
  //totaltime = (double)(finish - start);
  //std::cout << "\nRANSAC的运行时间为" << totaltime << "ms！" << std::endl;

  // print the coefficients of the plane
  double pa = coefficients->values[0];
  double pb = coefficients->values[1];
  double pc = coefficients->values[2];
  double pd = coefficients->values[3];
	      
  // compute errors
  double error = 0.00; // sum of the distances (points to plane)
  double ave = 0.00; // the average of the distance
  for (int i = 0; i < 4 * K * K; ++i)
    {
      error += abs(pa * fit_cloud->points[i].x + pb * fit_cloud->points[i].y + pc * fit_cloud->points[i].z + pd);
      ave += abs(pa * fit_cloud->points[i].x + pb * fit_cloud->points[i].y + pc * fit_cloud->points[i].z);
    }
  ave /= (4 * K * K);
  error /= sqrt(pa * pa + pb * pb + pc * pc);
  error /= (4 * K * K);	      
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

void BiCamera::FitLine(PC::Ptr cloud) 
{
  //clock_t start, finish;
  //double totaltime;
  //start = clock();
  
  // fitting a plane(use point clouds)
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_LINE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  
  seg.setInputCloud (cloud);

  cout <<"ok\n";
  
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a LINE model for the given dataset.");
    }


  //finish = clock();
  //totaltime = (double)(finish - start);
  //std::cout << "\nRANSAC的运行时间为" << totaltime << "ms！" << std::endl;

  // print the coefficients of the plane
  for (int i = 0; i < 3; i++)
    cout << coefficients->values[i] << endl;
  // double pa = coefficients->values[0];
  // double pb = coefficients->values[1];
  // double pc = coefficients->values[2];
  // double pd = coefficients->values[3];
  
}

void BiCamera::Init()
{
  // ros publish
  pub = nh.advertise<PC> ("points_temp", 1);
  pub2 = nh.advertise<PC> ("points_test", 1);

  // set and open camera
  sel = CAM_STEREO_752X480_LD_30FPS;
  c = new movesense::MoveSenseCamera(sel);

  //  # if use camera
  // if(!(movesense::MS_SUCCESS == c->OpenCamera()))
  //   {
  //     std::cout << "Open Camera Failed!" << std::endl;
  //     std::exit(1);
  //   }
  width  = 752;
  height = 480;
  len  = width * height * 2;
  img_data = new unsigned char[len];
}

void BiCamera::ProcessTemplate() //
{
  char left_name[60];
  char disp_name[60];
  Mat left, disp;
  for (int i = 0; i < TEMPNUM; i++)
    {
      // create temperary point clouds storaging data
      PC::Ptr temp_cloud(new PC);

      temp_cloud->height = height;
      temp_cloud->width = width;
      temp_cloud->is_dense = false;
      temp_cloud->points.resize(temp_cloud->width * temp_cloud->height);

      // read image
      //if (i == 0)
      //      	{
      sprintf(left_name, "img/template/left_%d.jpg", i + 1);
      sprintf(disp_name, "img/template/disp_%d.jpg", i + 1);
      //	}
      //else
      //	{
      // sprintf(left_name, "img/test/left_%d.jpg", i + 6);
      // sprintf(disp_name, "img/test/disp_%d.jpg", i + 6);
      //	}
      left = imread(left_name, 0);
      disp = imread(disp_name, 0);
      
      DepthImageToPc(disp, temp_cloud); // depth image convert to point clouds

      PC::Ptr cloud_filtered(new PC);
      FilterPc(temp_cloud, cloud_filtered); // filter point clouds
      RemoveNoise(cloud_filtered); // remove ground, celling, obstacles
      
      temp_cloud_ptr.push_back(cloud_filtered);
    }
  cout << "ProcessTemplate over.\n";
}

void BiCamera::ProcessTest(Mat& disp) 
{
  PC::Ptr test_cloud(new PC);
  test_cloud->height = height;
  test_cloud->width = width;
  test_cloud->is_dense = false;
  test_cloud->points.resize(test_cloud->width * test_cloud->height);

  DepthImageToPc(disp, test_cloud); // depth image convert to point clouds

  PC::Ptr cloud(new PC);
  cloud->height = height;
  cloud->width = width;
  cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);
  FilterPc(test_cloud, cloud); // filter point clouds

  RemoveNoise(cloud); // remove ground, celling, obstacles
  
  // match two point clouds using ICP
  PC::Ptr output(new PC);
  output->header.frame_id = "map";
  float min = FLT_MAX;
  int min_index = 0;
  for (int i = 0; i < TEMPNUM; i++)
    {
      // clock_t start, finish;
      // double totaltime;
      // start = clock();
      
      ICP_result result = MatchTwoPc(temp_cloud_ptr[i], cloud, output);
      
      // finish = clock();
      // totaltime = (double)(finish - start);
      // cout << "\n icp = " << totaltime / 1000.0 << "ms！" << endl;
      
      if (result.conv == true && result.score <= min)
	{
	  min = result.score;
	  min_index = i;
	}
    }
  
  // ROS_WARN("This image is similiar to disp_%d  score: %f\n", min_index + 1, min);
  printf("This image is similiar to disp_%d  score: %f\n", min_index + 1, min);
  cout << "ProcessTest over.\n";
}

void BiCamera::DepthImageToPc(Mat& img, PC::Ptr cloud) 
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
	if(d < 32 * 4)
	  d_real = d / 4.0;
	else
	  d_real = (d * 2 - 128) / 4.0;

	// compute (x, y, z)
	z = kBMultipyF / d_real;
	x = (u - kCu) * z / kF;
	y = (v - kCv) * z / kF;

	// storage data to cloud
	if (z > 1000 && z < 2000) // constraint z-axis
	  {
	    cloud->points[index].x = x / SCALE; // divide SCALE in order to accelerate 
	    cloud->points[index].y = y / SCALE;
	    cloud->points[index].z = z / SCALE;
	    index++;
	 }
      }
  cloud->points.resize(index); // remove no-data points
}

void BiCamera::RemoveNoise(PC::Ptr cloud)
{
  // search for connected domain(cluster) to remove ground or wall
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (5); // tolerant distance
  ec.setMinClusterSize (1000); // minimum cluster size
  ec.setMaxClusterSize (20000); // maximum cluster size
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  cout << cluster_indices.size() << endl;

  // find cluster which has minimum z-range 
  float zrange_min = FLT_MAX;
  int index = 0;
  int min_index = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    PC::Ptr cloud_cluster (new PC);
    float z_max = 0.0, z_min = FLT_MAX;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
	cloud_cluster->points.push_back (cloud->points[*pit]);
	if (cloud->points[*pit].z < z_min) z_min = cloud->points[*pit].z;
	if (cloud->points[*pit].z > z_max) z_max = cloud->points[*pit].z;
      }
    
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // compute z-axis range of this cluster
    float z_range = z_max - z_min;
    if (z_range <= zrange_min)
      {
 	// *cloud = *cloud_cluster; // if here, some strange situations exist.(some strange noises)
     	zrange_min = z_range;
     	min_index = index;
      }

    // if (index == 1) *cloud = *cloud_cluster;
    ++index;

    cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << endl;
  }

  // assign minimum z-range cluster to cloud
  index = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    if (index == min_index)
      {
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

void BiCamera::FilterPc(PC::Ptr cloud, PC::Ptr cloud_filtered)
{ 
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud (cloud);
  vg.setLeafSize (1, 1, 1);
  vg.filter (*cloud_filtered);
  cout << cloud->points.size() << "   " << cloud_filtered->points.size() << endl;
}

void BiCamera::Normalize(PC::Ptr cloud, PC::Ptr cloud_normalized)
{
  pcl::PCA<pcl::PointXYZ> pca(*cloud);
  // pca.setInputCloud (cloud->makeShared ());
  Eigen::Matrix3f eigen_vector = pca.getEigenVectors();
  cout << endl << eigen_vector << endl;
  float tan_theta = eigen_vector(1, 0) / eigen_vector(0, 0);
  cout << "arctan  " << atan (tan_theta) * 180 / M_PI << endl;
}

ICP_result BiCamera::MatchTwoPc(PC::Ptr target, PC::Ptr source, PC::Ptr output) // ICP
{
  PC::Ptr src(new PC);  
  PC::Ptr tgt(new PC);  
  
  tgt = target;  
  src = source;  
  
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;  
  icp.setMaxCorrespondenceDistance(0.05); // this param is very important!!!  
  icp.setTransformationEpsilon(1e-10); // if difference between two transformation matrix smaller than threshold, converge
  icp.setEuclideanFitnessEpsilon(0.01); // if sum of MSE smaller than threshold, converge
  icp.setMaximumIterations(100); // if iteration smaller than threshold, converge 
  
  icp.setInputSource(src);  
  icp.setInputTarget(tgt);  
  icp.align(*output);  
  cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;  
  
  output->resize(tgt->size() + output->size());  
  for (int i = 0; i < tgt->size(); i++)  
    output->push_back(tgt->points[i]); // merge two points 
  //cout << "After registration using ICP:" << output->size() << endl;
  cout << icp.getFinalTransformation() << endl;

  ICP_result result;
  result.conv = icp.hasConverged();
  result.score = icp.getFitnessScore();
  return result;
}

void BiCamera::ShowRviz()
{
  PC::Ptr msg (new PC);
  PC::Ptr msg2 (new PC);

  msg->header.frame_id = "map";
  msg->height = temp_cloud_ptr[RVIZ-1]->points.size();
  msg->width = 1;
  msg->is_dense = true;
  msg->points.resize(temp_cloud_ptr[RVIZ-1]->points.size()); // necessary

  // msg2->header.frame_id = "map";
  // msg2->height = temp_cloud_ptr[RVIZ-2]->points.size();
  // msg2->width = 1;
  // msg2->is_dense = true;
  // msg2->points.resize(temp_cloud_ptr[RVIZ-2]->points.size());
      
  for (size_t i = 0; i < msg->points.size (); ++i)
   {
     msg->points[i].x = temp_cloud_ptr[RVIZ-1]->points[i].x / 255.0 * SCALE;
     msg->points[i].y = temp_cloud_ptr[RVIZ-1]->points[i].y / 255.0 * SCALE;
     msg->points[i].z = temp_cloud_ptr[RVIZ-1]->points[i].z / 255.0 * SCALE;
   }

  PC::Ptr cloud_transformed (new PC);
  cloud_transformed->header.frame_id = "map";
  Transform(msg, cloud_transformed);

  // PC::Ptr output(new PC);
  // output->header.frame_id = "map";
  // MatchTwoPc(msg, trans, output);

  // for (size_t i = 0; i < msg2->points.size (); ++i)
  //   {
  //     msg2->points[i].x = temp_cloud_ptr[RVIZ-2]->points[i].x / 255.0 * SCALE;
  //     msg2->points[i].y = temp_cloud_ptr[RVIZ-2]->points[i].y / 255.0 * SCALE;
  //     msg2->points[i].z = temp_cloud_ptr[RVIZ-2]->points[i].z / 255.0 * SCALE;
  //   }

  PC::Ptr cloud_normalized (new PC);
  cloud_normalized->header.frame_id = "map";
  Normalize(cloud_transformed, cloud_normalized);

  //FitLine(cloud_transformed); // some errors
      
  pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
  pub.publish (msg);     
  pcl_conversions::toPCL(ros::Time::now(), cloud_transformed->header.stamp);
  pub2.publish (cloud_transformed);
}

void BiCamera::Transform(PC::Ptr cloud, PC::Ptr trans)
{
  float theta = M_PI/4; // rotate pi/8
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  
  // Define a translation of 2.5 meters on the x axis.
  transform_2.translation() << 1.0, 0.0, 0.0; // transform (1, 0, 0)
  
  // theta radians arround Z axis
  transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
  
  // Print the transformation
  printf ("using an Affine3f\n");
  std::cout << transform_2.matrix() << std::endl;

  // Executing the transformation
  pcl::transformPointCloud (*cloud, *trans, transform_2);

  cout << "original  " << theta / M_PI * 180 << endl;
}

void BiCamera::Run()
{
  Mat left, disp;
  
  // create filter_point which points to all template pointclouds
  ProcessTemplate(); // preprocess template
  
  loop_rate = new ros::Rate(4);
  flag = false;

  while (nh.ok())
    {
      // # if use camera
      // c->GetImageData(img_data, len);
      // Mat left(height, width, CV_8UC1), disp(height, width, CV_8UC1); // disp is the disparity map
      // for(int i = 0 ; i < height; i++)
      // 	{
      // 	  memcpy(left.data + width * i, img_data + (2 * i) * width, width);
      // 	  memcpy(disp.data + width * i, img_data + (2 * i + 1) * width, width);
      // 	}

      //use mid-filter for disp
      // medianBlur(disp, disp, 5);

      // compute time
      clock_t start, finish;
      double totaltime;
      start = clock();
  
      char left_name[60];
      char disp_name[60];
      sprintf(left_name, "img/test/left_%d.jpg", TIM);
      sprintf(disp_name, "img/test/disp_%d.jpg", TIM);
      left = imread(left_name, 0);
      disp = imread(disp_name, 0);
      
      ProcessTest(disp); // estimate test poses

      ShowRviz();
	
      ros::spinOnce ();
      loop_rate->sleep (); // private
      
      finish = clock();
      totaltime = (double)(finish - start);
      cout << "\n run time = " << totaltime / 1000.0 << "ms！" << endl;

      char key = waitKey(50);
      if(key == 'q') // quit
  	break;
      // # if use camera
      // else if (key == 's') // save
      // 	{
      // 	  sprintf(left_name, "left_%d.jpg", num);
      // 	  sprintf(disp_name, "disp_%d.jpg", num);
      // 	  num++;
      // 	  imwrite(left_name, left);
      // 	  imwrite(disp_name, disp);
      // 	}
    }
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  
  BiCamera cam;
  cam.Init();
  cam.Run();

  // DBSCAN::ClusterData cl_d = DBSCAN::gen_cluster_data( 3, 100 );
  // DBSCAN dbs(0.1, 5, 1); // threshold 0.1, minPt 5, thread 1
  // dbs.fit( cl_d );
  // cout << dbs << endl;
}
