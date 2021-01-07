#include "CameraClient.h"
#include <opencv2/imgcodecs.hpp>
#include "sample.h"

//In this sample, we will show how to use a camera to take a 2d color image and save it on the disk
int sample::get2DImage()
{
	//Before we do anything, we always need to connect to camera by ip.
	CameraClient camera;
	std::string error;
	// Camera ip should be modified to actual ip address.
	const std::string cameraIp = "192.168.3.137";
	if (!camera.setIp(cameraIp)) return -1; //return -1 if connection to camera fails

	std::cout << "Camera IP: " << camera.getCameraIp() << std::endl
		<< "Camera ID: " << camera.getCameraId() << std::endl
		<< "Version: " << camera.getCameraVersion() << std::endl; //get and print some information about camera device

	cv::Mat color = camera.captureColorImg(); //capture a 2d image and it will be stored as cv matrix
	if (color.empty()) std::cout << "empty error" << std::endl;
	else
	{
		cv::imwrite("D:/test/2d.jpg", color); //save sd image file on the disk
	}

	return 0;
}