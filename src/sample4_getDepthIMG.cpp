#include "CameraClient.h"
#include <opencv2/imgcodecs.hpp>
#include "sample.h"

//In this sample, we will show how to use a camera to take a depth image and save it on the disk

int sample::getDepthImage()
{
	//Before we do anything, we always need to connect to camera by ip.
	CameraClient camera;
	std::string error;
	// Camera ip should be modified to actual ip address.
	const std::string cameraIp = "192.168.3.146";
	if (!camera.connect(cameraIp)) return -1; //return -1 if connection to camera fails
	std::cout << "Camera IP: " << camera.getCameraIp() << std::endl
		<< "Camera ID: " << camera.getCameraId() << std::endl
		<< "Version: " << camera.getCameraVersion() << std::endl; //get and print some information about camera device

	cv::Mat depth = camera.captureDepthImg(); //capture a depth image and it will be stored as cv matrix
	if (depth.empty()) std::cout << "empty error" << std::endl;
	else
	{
		cv::imwrite("D:/test/depth.png", depth); //save sd image file on the disk
	}

	return 0;
}