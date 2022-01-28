#include "CameraClient.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/persistence.hpp>
#include "sample.h"

//In this sample, we will show how to use a camera to take a 2d color image and save it on the disk.
int sample::ImgAndCloud()
{
	//Before we do anything, we always need to connect to camera by ip.
	CameraClient camera;
	std::string error;
	// Camera ip should be modified to actual ip address.
	const std::string cameraIp = "192.168.3.19";
	if (!camera.connect(cameraIp)) return -1; //return -1 if connection to camera fails
	std::cout 
		<< "Camera ID: " << camera.getCameraId() << std::endl
		<< "Version: " << camera.getCameraVersion() << std::endl 
		<< "Color Image Size: " << camera.getColorImgSize() << std::endl
		<< "Depth Image Size: " << camera.getDepthImgSize() << std::endl; //Get some information about camera device.

	cv::Mat color = camera.captureColorImg(); //Capture a 2d image which will be stored as cv matrix.
	if (color.empty()) std::cout << "The color image is empty!" << std::endl;
	else
	{
		cv::imwrite("d://color.jpg", color); //Save the color image to the disk.
	}

	cv::Mat depth = camera.captureDepthImg(); //Capture a depth image which will be stored as cv matrix.
	if (depth.empty()) std::cout << "The depth image is empty!" << std::endl;
	else
	{
		std::string savePath("d://images.yml"); //Set the target path you want to save the depth image ending in yml.
		cv::FileStorage fs(savePath, cv::FileStorage::WRITE); 
		write(fs, "depth", depth); //Save the depth image to the disk.
	}

	std::cout << "Generating point cloud image" << std::endl;
	const pcl::PointCloud<pcl::PointXYZRGB> rgbCloud = camera.captureRgbPointCloud();
	PointCloudTools::saveRgbPointCloud("d://rgbCloud.ply", rgbCloud); //It can be .ply or .pcd.

    // Read the images from saved files and show them.
	std::cout << "Rendering images......." << std::endl;
	cv::Mat savedColorImage = cv::imread("d://color.jpg"); 
	if (savedColorImage.empty()) std::cout << "Fail to save the color image!" << std::endl;
	else {
		cv::namedWindow("saved color image", cv::WINDOW_NORMAL); //Create a window for showing images.
		cv::imshow("saved color image", savedColorImage);
		cv::waitKey(10000); //Show the color image and wait for 10s or key inputs.
	}
	cv::destroyAllWindows();

	return 0;
}


