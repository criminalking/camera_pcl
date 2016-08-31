#include "PoseRecognition.h"


BiCamera::~BiCamera()
{
  delete loop_rate;
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

  // get the coefficients of the plane
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


void BiCamera::Init()
{
  // ros publish
  pub = nh.advertise<PC> ("points_compute", 1); 
  pub2 = nh.advertise<PC> ("points_right", 1);
  pub3 = nh.advertise<PC> ("points_test", 1);

  // use camera
  // process_image.OpenCamera();
}


void BiCamera::Run()
{
  int num;
  int length = 9;
  int index = 0;

  // test some error images
  int face_index[9] = {9, 17, 18, 32, 34, 42, 52, 58, 63};

  // test all images
  //int face_index[length];
  //for (int i = 0; i < length; i++)
  //  face_index[i] = i + 1;
  
  loop_rate = new ros::Rate(4);
  
  // preprocess template
  while(true) 
    {
      if (ProcessTemplate() == true) break;
    }

  while (nh.ok())
    {
      // if use camera
      Mat left(height, width, CV_8UC1), disp(height, width, CV_8UC1); // disp is the disparity map  
      //process_image.GetImageFromCamera(left, disp);

      // read image
      if (index >= length) break;
      num = face_index[index];
	  
      process_image.GetImage(left, disp, num, TEST);
      imshow("left", left);
      imshow("disp", disp);

      char key = waitKey(10);
      if(key == 'q') // quit
  	break;
      else if (key == 's') // save
      	{
  	  // compute time
  	  clock_t start, finish;
  	  double totaltime;
  	  start = clock();
	  
	  if (ProcessTest(left, disp, num) == false) // estimate test poses
	    ROS_WARN("Sorry, no poses are found.\n");
	  ++index;
            
  	  finish = clock();
  	  totaltime = (double)(finish - start);
  	  cout << "\nrun time = " << totaltime / 1000.0 << "ms！" << endl;
	}

      ShowRviz();
      
      loop_rate->sleep ();
      ros::spinOnce ();
    }
}


bool BiCamera::ProcessTemplate() 
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

      // get a template image
      process_image.GetImage(left, disp, i + 1, TEMPLATE); 

      // median filtering
      medianBlur(disp, disp, 9);

      // depth image convert to point clouds
      DepthImageToPc(disp, cloud);
      
      // get people cluster
      if (get_people.TrackFace(left, disp, cloud, face_mid_point) == false) 
	return false;
      
      // normalize point clouds
      PC::Ptr cloud_normalized(new PC);
      Normalize(cloud, cloud_normalized);

      // compute z_range
      pcl::PointXYZ min_pt, max_pt;
      pcl::getMinMax3D(*cloud_normalized, min_pt, max_pt); // get minmum and maximum points in the z-axis
      float z_range = face_mid_point.z - min_pt.z;
      cout << "z_range: " << z_range << endl;
      if (z_range < 0) return false;
      else if (z_range < ZRANGE) // x-y pose(could ignore z-axis)
       	Projection(cloud_normalized); // project to xy-plane
      //else // x-y-z pose
      // 	Projection(cloud_normalized, 1); // project to yz-plane

      // store template point clouds
      temp_cloud_ptr.push_back(cloud_normalized); 
    }
  printf("ProcessTemplate over.\n");
  return true;
}


bool BiCamera::ProcessTest(Mat& left, Mat& disp, int num) 
{
  // create temperary point clouds storaging data
  PC::Ptr cloud(new PC);
  cloud->height = height;
  cloud->width = width;
  cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);

  // median filtering
  medianBlur(disp, disp, 9);

  // depth image convert to point clouds
  DepthImageToPc(disp, cloud);

  // get people cluster
  if (get_people.TrackFace(left, disp, cloud, face_mid_point) == false) 
    return false;

  // normalize point cloud
  PC::Ptr cloud_normalized (new PC);
  Normalize(cloud, cloud_normalized);

  // compute the distance from face to the frontest part of the human body
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud_normalized, min_pt, max_pt); // get minmum and maximum points in the z-axis
  float z_range = face_mid_point.z - min_pt.z;
  cout << "z_range: " << z_range << endl;
  
  // match two point clouds using ICP(xy-plane) or some tricks(xyz-space)
  PC::Ptr output(new PC);
  float min_value = FLT_MAX;
  int min_index = 0;
  if (z_range < ZRANGE) // pose in xy-plane
    {
      Projection(cloud_normalized); // project to xy-plane
      *cloud_rviz_3 = *cloud_normalized;
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
      if (min_value < 20)
	{
	  printf("This image is similiar to disp_%d  score: %f\n", min_index + 1, min_value);
	  *cloud_rviz_1 = *temp_cloud_ptr[min_index];
	  if (answer[num - 1] - 1 >= 0 && answer[num - 1] - 1 < 10)
	    *cloud_rviz_2 = *temp_cloud_ptr[answer[num - 1] - 1];
	  if (min_index + 1 == answer[num - 1]) printf("\n%d Right!!!!!!!!!!!!\n", num);
	  else printf("\n%d Wrong!!!!!!!!!!!!!\n", num);
	}
      else
	return false;
    }
  else // pose in xyz-space
    {
      Projection(cloud_normalized, 1); // project to yz-plane
      pcl::getMinMax3D(*cloud_normalized, min_pt, max_pt); // get minmum and maximum points in the z-axis
      float xyz_z_range = max_pt.z - min_pt.z;
      float xyz_y_range = max_pt.y - min_pt.y;
      float y_max = min_pt.y;
      float y_min = max_pt.y;
      float y_sum = 0.0;
      int y_num = 0;

      for (size_t i = 0; i < cloud_normalized->points.size (); ++i)
	{
	  if (cloud_normalized->points[i].z - min_pt.z > 8 && cloud_normalized->points[i].z - min_pt.z < 12) // get points in a interval  and search for the y_range
	    {
	      if (cloud_normalized->points[i].y < y_min) y_min = cloud_normalized->points[i].y;
	      else if (cloud_normalized->points[i].y > y_max) y_max = cloud_normalized->points[i].y;
	    }
	  else if (cloud_normalized->points[i].z - min_pt.z < xyz_z_range / 3)
	    {
	      y_sum += cloud_normalized->points[i].y;
	      ++y_num;
	    }
	}
      y_sum /= y_num;
      
      if (y_max - y_min > 0.5 * xyz_y_range)
	{
	  printf("This image is similiar to disp_9\n");
	  if (answer[num - 1] == 9) printf("\n%d Right!!!!!!!!!!!!\n", num);
	  else printf("\n%d Wrong!!!!!!!!!!!!!\n", num);
	}
      else
	{
	  if (y_sum > (min_pt.y + max_pt.y) / 2)
	    {
	      printf("This image is similiar to disp_10\n");
	      if (answer[num - 1] == 10) printf("\n%d Right!!!!!!!!!!!!\n", num);
	      else printf("\n%d Wrong!!!!!!!!!!!!!\n", num);
	    }
	  else
	    {
	      printf("This image is similiar to disp_8\n");
	      if (answer[num - 1] == 8) printf("\n%d Right!!!!!!!!!!!!\n", num);
	      else printf("\n%d Wrong!!!!!!!!!!!!!\n", num);
	    }
	}
    }
  return true;
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

  // convert depth image to point cloud (z-value has constraint)
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
	cloud->points[index].x = x; // divide SCALE in order to accelerate 
	cloud->points[index].y = y;
	cloud->points[index].z = z;
	index++;
      }
  cloud->points.resize(index);
}


void BiCamera::Normalize(PC::Ptr cloud, PC::Ptr cloud_normalized)
{
  // first: rotate using PCA
  // pcl::PCA<pcl::PointXYZ> pca(*cloud);
  // Eigen::Matrix3f eigen_vector = pca.getEigenVectors();
  // cout << endl << eigen_vector << endl;
  // float tan_theta = eigen_vector(1, 0) / eigen_vector(0, 0);
  // float theta = atan (tan_theta); // degree
  // cout << "theta " << theta * 180 / M_PI << endl;
  
  PC::Ptr cloud_transformed (new PC);
  *cloud_transformed = *cloud;

  // second: scale
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt); // get minmum and maximum points in the x-axis(actually y)
  float x_range = max_pt.x - face_mid_point.x; // range of x-axis(Attention: face_mid_point.x is negative)
  float scale = HEIGHT / x_range; // compression ratio
  float x_offset = (max_pt.x - x_range / 2) * scale; // middle of the x-axis * scale
  float y_offset = face_mid_point.y * scale; 
  
  // all points should multiple scale
  for (size_t i = 0; i < cloud->points.size(); ++i)
    {
      cloud_transformed->points[i].x *= scale;
      cloud_transformed->points[i].y *= scale;
      cloud_transformed->points[i].z *= scale;
    }
  face_mid_point.x *= scale;
  face_mid_point.y *= scale;
  face_mid_point.z *= scale;  
  
  // third: translate
  Eigen::Matrix3d m(1, 3);
  m << -x_offset, -y_offset, 0.0;
  Transform(cloud_transformed, cloud_normalized, 0, m);
  face_mid_point.x = face_mid_point.x - x_offset;
  face_mid_point.y = face_mid_point.y - y_offset;
}


void BiCamera::Projection(PC::Ptr cloud, int flag)
{
  if (flag == 1)
    for (size_t i = 0; i < cloud->points.size(); ++i)
      cloud->points[i].x = 0;
  else if (flag == 2)
    for (size_t i = 0; i < cloud->points.size(); ++i)
      cloud->points[i].y = 0;
  else if (flag == 3)
    for (size_t i = 0; i < cloud->points.size(); ++i)
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
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; // method 1
  //pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; // method 2
  icp.setMaxCorrespondenceDistance(kIcpDistance); // this param is very important!!!  
  icp.setTransformationEpsilon(1e-10); // if difference between two transformation matrix smaller than this threshold, converge
  icp.setEuclideanFitnessEpsilon(0.01); // if sum of MSE smaller than threshold, converge
  icp.setMaximumIterations(100); // if iteration smaller than threshold, converge
  
  icp.setInputSource(source);  
  icp.setInputTarget(target);  
  icp.align(*output);  
  cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;  

  //output->resize(target->size() + output->size());  
  //for (int i = 0; i < target->size(); ++i)  
  //  output->push_back(target->points[i]); // merge two points 
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
  PC::Ptr msg3 (new PC);

  msg->header.frame_id = "map"; // necessary
  msg->height = cloud_rviz_1->points.size();
  msg->width = 1;
  msg->is_dense = true;
  msg->points.resize(cloud_rviz_1->points.size()); // necessary

  msg2->header.frame_id = "map"; // necessary
  msg2->height = cloud_rviz_2->points.size();
  msg2->width = 1;
  msg2->is_dense = true;
  msg2->points.resize(cloud_rviz_2->points.size()); // necessary

  msg3->header.frame_id = "map"; // necessary
  msg3->height = cloud_rviz_3->points.size();
  msg3->width = 1;
  msg3->is_dense = true;
  msg3->points.resize(cloud_rviz_3->points.size()); // necessary

  for (size_t i = 0; i < msg->points.size(); ++i)
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

  for (size_t i = 0; i < msg3->points.size (); ++i)
    {
      msg3->points[i].x = cloud_rviz_3->points[i].x / 255.0 * SCALE;
      msg3->points[i].y = cloud_rviz_3->points[i].y / 255.0 * SCALE;
      msg3->points[i].z = cloud_rviz_3->points[i].z / 255.0 * SCALE;
    }
      
  pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
  pub.publish (msg);     
  pcl_conversions::toPCL(ros::Time::now(), msg2->header.stamp);
  pub2.publish (msg2);
  pcl_conversions::toPCL(ros::Time::now(), msg3->header.stamp);
  pub3.publish (msg3);
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  
  BiCamera cam;
  cam.Init();
  cam.Run();
}
