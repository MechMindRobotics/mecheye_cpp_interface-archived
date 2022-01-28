#include "CameraClient.h"
#include "sample.h"

int sample::parameter()
{
	//Before we do anything, we always need to connect to camera by IP.
	CameraClient camera;
	std::string error;
	// The camera IP should be modified to actual IP address.
	const std::string cameraIp = "192.168.3.19";
	if (!camera.connect(cameraIp)) return -1; //return -1 if connection to camera fails
	std::cout << "Camera ID: " << camera.getCameraId() << std::endl
		<< "Version: " << camera.getCameraVersion() << std::endl; //get and print some information about camera device
	camera.getCameraIntri();

	std::cout << camera.setParameter("scan2dExposureMode",0) << std::endl; //set exposure mode to Timed
	std::cout << camera.getParameter("scan2dExposureMode", error) <<  error<< std::endl;
	std::cout << camera.setParameter("scan2dExposureTime", 20) << std::endl; //set exposure time to 20ms 
	std::cout << camera.getParameter("scan2dExposureTime", error) << error << std::endl;

	 //std::vector<int> roi{ 500, 500, 100, 100 }; // roi: { Height, Width, X, Y}
	 //std::cout << camera.setParameter("roi", roi) << std::endl; //set roi
	 //std::cout << camera.getParameter("roi", error) << error << std::endl;
	return 0;
}