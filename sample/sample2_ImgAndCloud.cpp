#include "CameraClient.h"
#include <opencv2/imgcodecs.hpp>
#include "sample.h"

//In this sample, we will show how to use a camera to take a 2d color image and save it on the disk
int sample::ImgAndCloud()
{
	//Before we do anything, we always need to connect to camera by ip.
	CameraClient camera;
	std::string error;
	// Camera ip should be modified to actual ip address.
	const std::string cameraIp = "192.168.3.226";
	if (!camera.connect(cameraIp)) return -1; //return -1 if connection to camera fails

	std::cout 
		<< "Camera ID: " << camera.getCameraId() << std::endl
		<< "Version: " << camera.getCameraVersion() << std::endl; //get and print some information about camera device

	cv::Mat color = camera.captureColorImg(); //capture a 2d image and it will be stored as cv matrix
	if (color.empty()) std::cout << "empty error" << std::endl;
	else
	{
		cv::imwrite("D:/test/2d.jpg", color); //save sd image file on the disk
	}

	cv::Mat depth = camera.captureDepthImg(); //capture a depth image and it will be stored as cv matrix
	if (depth.empty()) std::cout << "empty error" << std::endl;
	else
	{
		cv::imwrite("D:/test/depth.png", depth); //save sd image file on the disk
	}

	const pcl::PointCloud<pcl::PointXYZRGB> rgbCloud = camera.captureRgbPointCloud();
	PointCloudTools::saveRgbPointCloud("D:/test/rgbCloud.ply", rgbCloud); //can be .ply or .pcd, both works.

	return 0;
}