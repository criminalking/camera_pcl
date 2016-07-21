#include "example.h"

BiCamera::~BiCamera()
{
  delete c;
  delete loop_rate;
}

void BiCamera::FitPlane(PC::Ptr cloud,PC::Ptr fit_cloud) 
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
      std::cout << pa << " " << pb << " " << pc << " " << pd << std::endl;
      std::cout << "error = " << error << " mm  ";
      std::cout << "ave = " << ave << " mm  ";
      // show z-axis distance
      std::cout << "distance = " << distance << " mm" << std::endl;

      // write in .txt
      std::ofstream out;
      out.open("out.txt", std::ios::app);
      out << error << " " << (ave + distance) / 2 << "\n";
      out.close();

      // write in .txt
      std::ofstream ti;
      ti.open("time.txt", std::ios::app);
      ti << K << " " << totaltime << "\n";
      ti.close();
    }
}

void BiCamera::Init(PC::Ptr cloud, PC::Ptr fit_cloud)
{
  // ros publish
  pub = nh.advertise<PC> ("points2", 1);

  // set and open camera
  sel = CAM_STEREO_752X480_LD_30FPS;
  c = new movesense::MoveSenseCamera(sel);
  if(!(movesense::MS_SUCCESS == c->OpenCamera()))
    {
      std::cout << "Open Camera Failed!" << std::endl;
      std::exit(1);
    }
  width  = 752;
  height = 480;
  len  = width * height * 2;
  img_data = new unsigned char[len];

  // set point clouds
  cloud->header.frame_id = "map";
  cloud->height = K * 2;
  cloud->width = K * 2;
  cloud->is_dense = true;
  cloud->points.resize(cloud->width * cloud->height);

  fit_cloud->height = K * 2;
  fit_cloud->width = K * 2;
  fit_cloud->is_dense = true;
  fit_cloud->points.resize(fit_cloud->width * fit_cloud->height);
}

void BiCamera::DepthImageToPc(cv::Mat disp, PC::Ptr cloud) 
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
  for (int u = height / 2 - K; u < height / 2 + K; ++u)
    for (int v = width / 2 - K; v < width / 2 + K; ++v)
      {
	int d = disp.at<uchar>(u,v);
	if(d < 32 * 4)
	  d_real = d / 4.0;
	else
	  d_real = (d * 2 - 128) / 4.0;

	// compute (x, y, z)
	x = (u - kCu) * z / kF;
	y = (v - kCv) * z / kF;
	z = kBMultipyF / d_real;

	// show in rviz
	cloud->points[index].x = x / 255.0f; // /255.0f is just for show in rviz
	cloud->points[index].y = y / 255.0f;
	cloud->points[index].z = z / 255.0f;
	index++;
      }
}

void BiCamera::Run(PC::Ptr cloud, PC::Ptr fit_cloud)
{
  loop_rate = new ros::Rate(4);
  flag = false;
  
  while (nh.ok())
    { 
      c->GetImageData(img_data, len);
      if(len > 0)
	{
	  cv::Mat left(height, width, CV_8UC1), disp(height, width, CV_8UC1); // disp is the disparity map
	  for(int i = 0 ; i < height; i++)
	    {
	      memcpy(left.data + width * i, img_data + (2 * i) * width, width);
	      memcpy(disp.data + width * i, img_data + (2 * i + 1) * width, width);
	    }
	  
	  // draw a rect(2K * 2K) in the middle of the image "left"
	  cv::rectangle(
			left,
			cv::Rect(width / 2 - K, height / 2 - K, 2 * K, 2 * K),
			cv::Scalar(255, 255, 255)
			);
      
	  cv::imshow("left",left);
	  cv::imshow("disp",disp);

	  // save the image left and disp
	  //cv::imwrite( "left.jpg", left );
	  //cv::imwrite( "disp.jpg", disp );
	
	  //use mid-filter for disp
	  //cv::medianBlur(disp, disp, 5);
	
	  DepthImageToPc(disp, cloud);

	  if (flag == true) // this frame should be computed
	    {
	      FitPlane(cloud, fit_cloud);
	    }
	  
	  char key = cv::waitKey(100);
	  if(key == 'q') // quit
	    break;
	  else if (key == 'o') // start to fit plane
	    flag = true;
	  else if (key == 'c') // close to fit plane
	    flag = false;
	}
      
      pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
      pub.publish(cloud);
      ros::spinOnce ();
      loop_rate->sleep (); // private
    }
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  PC::Ptr cloud(new PC);
  PC::Ptr fit_cloud(new PC);

  BiCamera cam;
  cam.Init(cloud, fit_cloud);
  cam.Run(cloud, fit_cloud);
}
