#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#include <string>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>

#define CUSTOM_REPROJECT

using namespace std;

/*//This function creates a PCL visualizer, sets the point cloud to view and returns a pointer
boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
  viewer->addCoordinateSystem ( 1.0 );
  viewer->initCameraParameters ();
  return (viewer);
}*/

int main( int argc, char** argv )
{
	int showResult = 0;
	if (argc == 5)					//Check arguments
	{
		if (strcmp(argv[4], "-sr") == 0)
			showResult = 1;
		else {
			cout << "Ivalid parameter: " << argv[4] << endl;
			return -1;
		}
	}
	else
	{
		if (argc != 4)
		{
			cerr << "Usage: " << argv[0] << " <rgb-image-filename> <disparity-image-filename> <path-to-Q-matrix>" << endl;
			return 1;
		}
	}
  
	char nome[20];
	char c = argv[1][0];
	for(int i = 0; c != '.'; i++)
	{
		nome[i] = c;
		c = argv[1][i+1];
		nome[i+1] = '\0';
	}
	strcat(nome, "_pont_cloud.txt");
	//cout << nome << endl;

  
	  ofstream file;
	  file.open (nome);
	  
	  //Load Matrix Q
	  std::cout << "Reading matrix in file " << argv[3] << " ...";
	  cv::FileStorage fs(argv[3], cv::FileStorage::READ);
	  cv::Mat Q;
	  
	  fs["Q"] >> Q;
	  
	  //If size of Q is not 4x4 exit
	  if (Q.cols != 4 || Q.rows != 4)
	  {
	    std::cerr << "\nERROR: Could not read matrix Q (doesn't exist or size is not 4x4)" << std::endl;
	    return 1;
	  }

	#ifdef CUSTOM_REPROJECT
	  //Get the interesting parameters from Q
	  double Q03, Q13, Q23, Q32, Q33;
	  Q03 = Q.at<double>(0,3);
	  Q13 = Q.at<double>(1,3);
	  Q23 = Q.at<double>(2,3);
	  Q32 = Q.at<double>(3,2);
	  Q33 = Q.at<double>(3,3);
	  
	  //std::cout << "Q(0,3) = "<< Q03 <<"; Q(1,3) = "<< Q13 <<"; Q(2,3) = "<< Q23 <<"; Q(3,2) = "<< Q32 <<"; Q(3,3) = "<< Q33 <<";" << std::endl;
	  
	#endif  
	    std::cout << " OK." << std::endl;
	  
	  //std::cout << "Read matrix in file " << argv[3] << std::endl;

	  //Show the values inside Q (for debug purposes)
	  /*
	  for (int y = 0; y < Q.rows; y++)
	  {
	    const double* Qy = Q.ptr<double>(y);
	    for (int x = 0; x < Q.cols; x++)
	    {
	      std::cout << "Q(" << x << "," << y << ") = " << Qy[x] << std::endl;
	    }
	  }
	  */
	  
	  //Load rgb-image
	  cv::Mat img_rgb = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
	  if (img_rgb.data == NULL)
	  {
	    std::cerr << "ERROR: Could not read rgb-image: " << argv[1] << std::endl;
	    return 1;
	  }
	  
	  //Load disparity image
	  cv::Mat img_disparity = cv::imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
	  if (img_disparity.data == NULL)
	  {
	    std::cerr << "ERROR: Could not read disparity-image: " << argv[2] << std::endl;
	    return 1;
	  }
	  
	  //Both images must be same size
	  if (img_rgb.size() != img_disparity.size())
	  {
	    std::cerr << "ERROR: rgb-image and disparity-image have different sizes " << std::endl;
	    return 1;
	  }
	  
	  //Show both images (for debug purposes)
	/*  cv::namedWindow("rgb-image");
	  cv::namedWindow("disparity-image");
	  cv::imshow("rbg-image", img_rgb);
	  cv::imshow("disparity-image", img_disparity);
	  std::cout << "Press a key to continue..." << std::endl;
	  cv::waitKey(0);
	  cv::destroyWindow("rgb-image");
	  cv::destroyWindow("disparity-image");
	*/

	#ifndef CUSTOM_REPROJECT
	  //Create matrix that will contain 3D corrdinates of each pixel
	  cv::Mat recons3D(img_disparity.size(), CV_32FC3);
	  
	  //Reproject image to 3D
	  std::cout << "Reprojecting image to 3D..." << std::endl;
	  cv::reprojectImageTo3D( img_disparity, recons3D, Q, false, CV_32F );
	#endif  
	  //Create point cloud and fill it
	  //std::cout << "Creating Point Cloud..." <<std::endl;
	  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	  
	  double px, py, pz;
	  uchar pr, pg, pb;
	  
	  for (int i = 0; i < img_rgb.rows; i++)
	  {
	    uchar* rgb_ptr = img_rgb.ptr<uchar>(i);
	#ifdef CUSTOM_REPROJECT
	    uchar* disp_ptr = img_disparity.ptr<uchar>(i);
	#else
	    double* recons_ptr = recons3D.ptr<double>(i);
	#endif
	    for (int j = 0; j < img_rgb.cols; j=j+10)
	    {
	      //Get 3D coordinates
	#ifdef CUSTOM_REPROJECT
	      uchar d = disp_ptr[j];
	      //if ( d == 0 ) continue; //Discard bad pixels
	      if ( d > 500 || d == 0) continue; //Discard bad pixels
	      //if ( d > 100 || d < 30 || d == 0) continue; //Discard bad pixels
	    //cout << i << " " << j << " " << static_cast<double>(d) << endl;
	      double pw = -1.0 * static_cast<double>(d) * Q32 + Q33; 
	      px = static_cast<double>(j) + Q03;
	      py = static_cast<double>(i) + Q13;
	      pz = Q23;
	      
	      px = px/pw;
	      py = py/pw;
	      pz = pz/pw;

	#else
	      px = recons_ptr[3*j];
	      py = recons_ptr[3*j+1];
	      pz = recons_ptr[3*j+2];
	#endif
	      
	      //Get RGB info
	      pb = rgb_ptr[3*j];
	      pg = rgb_ptr[3*j+1];
	      pr = rgb_ptr[3*j+2];
	      
	      file << px << " " << py << " " << pz << " " << static_cast<double>(pr) << " " << static_cast<double>(pg) << " " << static_cast<double>(pb) << "\n";

	      /*if (showResult != 0)
	      {
		//Insert info into point cloud structure
		pcl::PointXYZRGB point;
		point.x = px;
		//cout << point.x << endl;
		point.y = py;
		point.z = pz;
		uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
			static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
		point.rgb = *reinterpret_cast<float*>(&rgb);
		point_cloud_ptr->points.push_back (point);
		
	      }*/
	    }
	  }
	  
	  /*if (showResult != 0)
	  {
		point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
		point_cloud_ptr->height = 1;
		
		//Create visualizer
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
		viewer = createVisualizer( point_cloud_ptr );

		//pcl::io::savePCDFileASCII ("test_pcd.pcd", *point_cloud_ptr);

		//Main loop
		while ( !viewer->wasStopped())
		{
		viewer->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		if( cvWaitKey(33) == 27 )
			break;
		}
	  }*/
	  
	  file.close();
	  return 0;
}
