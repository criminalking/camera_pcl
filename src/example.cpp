#include "example.h"

BiCamera::~BiCamera()
{
  delete loop_rate;
}

void BiCamera::FitPlane(PC::Ptr cloud,PC::Ptr fit_cloud) // need K
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

void BiCamera::Init()
{
  // ros publish
  pub = nh.advertise<PC> ("points2", 1);

  // camera frame configuration
  width  = 752;
  height = 480;
}

void BiCamera::ProcessTemplate() // FIXME: add point-list point to filter_cloud
{
  char left_name[60];
  char disp_name[60];
  for (int i = 0; i < TEMPNUM; i++)
    {
      // create temperary point clouds storaging data
      PC::Ptr temp_cloud(new PC);

      temp_cloud->header.frame_id = "map"; // set pointcloud which needs to be shown on rviz  
      temp_cloud->height = height;
      temp_cloud->width = width;
      temp_cloud->is_dense = false;
      temp_cloud->points.resize(temp_cloud->width * temp_cloud->height);

      // read image
      sprintf(left_name, "img/template/left_%d.jpg", i + 1);
      sprintf(disp_name, "img/template/disp_%d.jpg", i + 1);
      left = imread(left_name, 0);
      disp = imread(disp_name, 0);
      
      DepthImageToPc(disp, temp_cloud); // depth image convert to point clouds
      //imshow("aa", disp);
      
      // RemoveNoise(temp_cloud); // remove ground, celling, obstacles

      PC::Ptr filter_cloud(new PC);
      FilterPc(temp_cloud, filter_cloud); // filter point clouds
      temp_cloud_ptr.push_back(temp_cloud);
    }
}

void BiCamera::ProcessTest(Mat& disp) 
{
  PC::Ptr test_cloud(new PC);
  test_cloud->header.frame_id = "map"; // set pointcloud which needs to be shown on rviz
  test_cloud->height = height;
  test_cloud->width = width;
  test_cloud->is_dense = false;
  test_cloud->points.resize(test_cloud->width * test_cloud->height);

  DepthImageToPc(disp, test_cloud); // depth image convert to point clouds
  // RemoveNoise(test_cloud); // remove ground, celling, obstacles

  PC::Ptr cloud(new PC);
  cloud->header.frame_id = "map"; // set pointcloud which needs to be shown on rviz
  cloud->height = height;
  cloud->width = width;
  cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);
  FilterPc(test_cloud, cloud); // filter point clouds

  // match two point clouds using ICP
  PC::Ptr output(new PC);
  output->header.frame_id = "map";
  float score = 0.0, min = 1000000.0;
  int min_index = 0;
  for (int i = 0; i < TEMPNUM; i++)
    {
      score = MatchTwoPc(temp_cloud_ptr[i], test_cloud, output);
      if (score <= min)
	{
	  min = score;
	  min_index = i;
	}
    }
  ROS_WARN("This image is similiar to disp_%d  score: %d\n", min_index + 1, min);
  //cout << "This image is similiar to disp_" << min_index + 1 << "  score: " << min << endl;
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
	if (z > 1000 && z < 2000)
	  // && y / 255.0 > (-6) && y / 255.0 < 3)
	 {
	    cloud->points[index].x = x; // 255.0f is just for show in rviz
	    cloud->points[index].y = y;
	    cloud->points[index].z = z;
	    index++;
	 }
      }
  cloud->points.resize(index); // remove no-data points
}

void BiCamera::RemoveNoise(PC::Ptr cloud)
{
  // remove ground or wall
  // search for connected domain
  for (size_t i = 0; i < cloud->points.size (); ++i)
    {
      
    }
}

void BiCamera::FilterPc(PC::Ptr cloud, PC::Ptr filter_cloud)
{
  // filter_cloud = cloud;
  
  pcl::PCLPointCloud2::Ptr cloud_filtered_blob(new pcl::PCLPointCloud2), cloud_blob(new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*cloud, *cloud_blob); // convert PC::Ptr to pcl::PCLPointCloud2

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud_blob);
  sor.setLeafSize(15, 15, 15);
  sor.filter(*cloud_filtered_blob);

  // pcl::PassThrough pass;
  // pass.setInputCloud(cloud_blob);
  // pass.setFilterFieldName("z");
  // pass.setFilterLimits(0.0, 1.0);
  // //pass.setFilterLimitsNegative (true);
  // pass.filter(*cloud_filtered_blob);//滤波后的数据存储在cloud_filtered中
  
  pcl::fromPCLPointCloud2(*cloud_filtered_blob, *filter_cloud); // convert pcl::PCLPointCloud2 to PC::Ptr
  cout << cloud->points.size() << "   " << filter_cloud->points.size() << endl;
}

float BiCamera::MatchTwoPc(PC::Ptr target, PC::Ptr source, PC::Ptr output) // change source
{
  PC::Ptr src(new PC);  
  PC::Ptr tgt(new PC);  
  
  tgt = target;  
  src = source;  
  
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;  
  icp.setMaxCorrespondenceDistance(0.1); // this param is very important!!!  
  icp.setTransformationEpsilon(1e-10); // if difference between two transformation matrix smaller than threshold, converge
  icp.setEuclideanFitnessEpsilon(0.01); // if sum of MSE smaller than threshold, converge
  icp.setMaximumIterations(100); // if iteration smaller than threshold, converge 
  
  icp.setInputSource(src);  
  icp.setInputTarget(tgt);  
  icp.align(*output);  
  cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;  
  
  output->resize(tgt->size() + output->size());  
  for (int i = 0; i < tgt->size(); i++)  
    output->push_back(tgt->points[i]); //合并  
  //cout << "After registration using ICP:" << output->size() << endl;
  cout << icp.getFinalTransformation() << endl;
  return icp.getFitnessScore();
}

void BiCamera::Run()
{
  // create filter_point which points to all template pointclouds
  ProcessTemplate(); // preprocess template
  
  loop_rate = new ros::Rate(4);
  flag = false;
  
  int q = 3;
  q--;
  pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
  msg->header.frame_id = "map";
  msg->height = height;
  msg->width = width;
  msg->is_dense = false;
  msg->points.resize(msg->width * msg->height);  

  while (nh.ok())
    { 	
      //use mid-filter for disp
      //medianBlur(disp, disp, 5);

      char left_name[60];
      char disp_name[60];
      int i = 9;
      sprintf(left_name, "img/test/left_%d.jpg", i);
      sprintf(disp_name, "img/test/disp_%d.jpg", i);
      left = imread(left_name, 0);
      disp = imread(disp_name, 0);
      
      ProcessTest(disp); // estimate test poses

      for (size_t i = 0; i < msg->points.size (); ++i)
  	{
  	  msg->points[i].x = temp_cloud_ptr[q]->points[i].x / 255.0;
  	  msg->points[i].y = temp_cloud_ptr[q]->points[i].y / 255.0;
  	  msg->points[i].z = temp_cloud_ptr[q]->points[i].z / 255.0;
  	}

      char key = waitKey(100);
      if(key == 'q') // quit
  	break;

      //pcl_conversions::toPCL(ros::Time::now(), temp_cloud_ptr[0]->header.stamp);
      //pub.publish(temp_cloud_ptr[0]);
      
      pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
      pub.publish (msg);      
      ros::spinOnce ();
      loop_rate->sleep (); // private
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
