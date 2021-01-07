#include "CameraClient.h"
#include <opencv2/imgcodecs.hpp>
#include "sample.h"

//In this sample, we will show how to change 3D exposure times and every time's  exposure time.

int sample::getPointCloud()
{
	//Before we do anything, we always need to connect to camera by ip.
	CameraClient camera;
	std::string error;
	// Camera ip should be modified to actual ip address.
	const std::string cameraIp = "192.168.3.146";
	if (!camera.setIp(cameraIp)) return -1; //return -1 if connection to camera fails

	std::cout << "Camera IP: " << camera.getCameraIp() << std::endl
		<< "Camera ID: " << camera.getCameraId() << std::endl
		<< "Version: " << camera.getCameraVersion() << std::endl; //get and print some information about camera device

	const CameraIntri intri = camera.getCameraIntri(); //get intrinsic parameter of the camera
	const pcl::PointCloud<pcl::PointXYZ> cloud = camera.capturePointCloud(intri); //calculate point cloud and store in pcl::PointCloud
	PointCloudTools::savePointCloud("D:/test/cloud.ply", cloud); //save point cloud on the disk
	return 0;
}