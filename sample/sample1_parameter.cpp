#include "CameraClient.h"
#include "sample.h"

int sample::parameter()
{
	//Before we do anything, we always need to connect to camera by ip.
	CameraClient camera;
	std::string error;
	// Camera ip should be modified to actual ip address.
	const std::string cameraIp = "192.168.3.76";
	if (!camera.connect(cameraIp)) return -1; //return -1 if connection to camera fails
	std::string para = "exposure1"; //parameter name you want to get or set
	std::cout << "Camera IP: " << camera.getCameraIp() << std::endl
		<< "Camera ID: " << camera.getCameraId() << std::endl
		<< "Version: " << camera.getCameraVersion() << std::endl; //get and print some information about camera device
	
	std::cout << camera.setParamater(para, 10) << std::endl;
	//unit for exposure time is ms, return value of setParameter is string.If successful, the return string will be empty, otherwise it prints error message.
	std::cout<<"The value is "<<camera.getParameter(para, error)<<std::endl;
	if(!error.empty())  
		std::cout << "ERROR:" << error << std::endl;
	//get the value of a parameter. The "error" would be empty if successful.


	return 0;
}