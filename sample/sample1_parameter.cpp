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
	std::cout << "Camera IP: " << camera.getCameraIp() << std::endl
		<< "Camera ID: " << camera.getCameraId() << std::endl
		<< "Version: " << camera.getCameraVersion() << std::endl; //get and print some information about camera device
	//The following is all the parameters you can set
	std::cout << camera.setParameter("period", 20) << std::endl;
	std::cout << camera.setParameter("isNanoType", 0) << std::endl;
	std::cout << camera.setParameter("lightPower", 300) << std::endl;
	std::cout << camera.setParameter("syncExposure", 1) << std::endl;
	std::cout << camera.setParameter("exposure1", 0.3) << std::endl;
	std::cout << camera.setParameter("exposure2", 6) << std::endl;
	std::cout << camera.setParameter("exposure3", 6) << std::endl;
	std::cout << camera.setParameter("gain", 0) << std::endl;
	std::cout << camera.setParameter("useBinning", 0) << std::endl;
	std::cout << camera.setParameter("useColorHdr", 0) << std::endl;
	std::cout << camera.setParameter("camera2DExpTime", 40) << std::endl;
	std::cout << camera.setParameter("expectedGrayValue", 120) << std::endl;
	std::cout << camera.setParameter("sharpenFactor", 0) << std::endl;
	std::cout << camera.setParameter("contrastThres", 10) << std::endl;
	std::cout << camera.setParameter("strength", 5) << std::endl;
	std::cout << camera.setParameter("useMedianBlur", 1) << std::endl;
	std::cout << camera.setParameter("hasThinObject", 0) << std::endl;
	std::cout << camera.setParameter("lowerLimit", 800) << std::endl;
	std::cout << camera.setParameter("upperLimit", 1100) << std::endl;




	
	std::cout<<"The value is "<<camera.getParameter("camera2DExpTime", error)<<std::endl;
	if(!error.empty())  
		std::cout << "ERROR:" << error << std::endl;
	//get the value of a parameter. The "error" would be empty if successful.


	return 0;
}