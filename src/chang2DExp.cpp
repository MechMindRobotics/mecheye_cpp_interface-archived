#include "CameraClient.h"
#include <opencv2/imgcodecs.hpp>
#include "sample.h"

int sample::change2DExposureTime()
{
	//Before we do anything, we always need to connect to camera by ip.
	CameraClient camera;
	std::string error;
	// Camera ip should be modified to actual ip address.
	const std::string cameraIp = "192.168.3.167";
	if (!camera.setIp(cameraIp)) return -1; //return -1 if connection to camera fails
	std::string para = "camera2DExpTime"; //parameter name you want to get or set
	std::cout << "Camera IP: " << camera.getCameraIp() << std::endl
		<< "Camera ID: " << camera.getCameraId() << std::endl
		<< "Version: " << camera.getCameraVersion() << std::endl; //get and print some information about camera device

	//unit for exposure time is ms, return value of setParameter is string.If successful, the return string will be empty, otherwise it prints error message.
	std::cout<<camera.setParamater(para, 10)<<std::endl;
	
	//get the value of a parameter. The "error" would be empty if successful.
	std::cout<<camera.getParameter(para, error)<<std::endl;
	if(!error.empty())
	std::cout << "ERROR:" << error << std::endl;
	

	

	return 0;
}