#ifndef CAMERACLIENT_H
#define CAMERACLIENT_H

#include "ZmqClient.h"
#include "PointCloudTools.h"
#include "image.pb.h"
#include "cameraStatus.pb.h"

class CameraClient : public ZmqClient
{
public:
	// Connect to camera before call other functions.
	bool connect(const std::string& ip) { return setAddr(ip, kImagePort, 60000); }

	// Depth image type: CV_32FC1
	cv::Mat captureDepthImg();

	// Color image type: CV_8UC3
	// expTime : ms
	cv::Mat captureColorImg();  

	

	// Units of point cloud: meter
	pcl::PointCloud<pcl::PointXYZ> capturePointCloud();
	pcl::PointCloud<pcl::PointXYZRGB> captureRgbPointCloud();

	CameraIntri getCameraIntri();

	std::string getCameraId(); //get camera's id
	std::string getCameraIp(); //get ip address of camera
	std::string getCameraVersion(); //get the version of the camera
	std::string getParameter(const std::string paraName, std::string& error); //exposed API for getting camera's parameters
	std::string setParameter(const std::string paraName, double value); //exposed API for setting camera's parameters


private:
	mmind::Response sendRequest(int command, double value);
	mmind::Response sendRequest(int command, const std::string& value);
	mmind::CameraStatus getCameraStatus();
	std::string getCameraParameter(const std::string& propertyName, std::string& error);
	std::string setCameraParameter(const std::string& propertyName, double value);
	const uint16_t kImagePort = 5577;
};


#endif // CAMERACLIENT_H
