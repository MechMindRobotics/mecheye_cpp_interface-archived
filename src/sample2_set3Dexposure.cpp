#include "CameraClient.h"
#include "sample.h"

//In this sample, we will show how to change 3D exposure times and every time's  exposure time.

int sample::change3DExposureTime()
{
	//Before we do anything, we always need to connect to camera by ip.
	CameraClient camera;
	std::string error;
	// Camera ip should be modified to actual ip address.
	const std::string cameraIp = "192.168.3.146";
	if (!camera.connect(cameraIp)) return -1; //return -1 if connection to camera fails
	std::string paraName = "scanMultiplier"; //3d exposure times, value can be from 1 to 3
	std::cout << "Camera IP: " << camera.getCameraIp() << std::endl
		<< "Camera ID: " << camera.getCameraId() << std::endl
		<< "Version: " << camera.getCameraVersion() << std::endl; //get and print some information about camera device

	std::cout << camera.setParamater(paraName, 3) << std::endl;
	//return value of setParameter is string.If successful, the return string will be empty, otherwise it prints error message.
	std::cout << camera.setParamater("exposure1", 3) << std::endl;
	std::cout << camera.setParamater("exposure1", 5) << std::endl;
	std::cout << camera.setParamater("exposure1", 4) << std::endl;


	return 0;
}