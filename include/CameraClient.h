#ifndef CAMERACLIENT_H
#define CAMERACLIENT_H


#include "ZmqClient.h"
#include "PointCloudTools.h"
#include "json/json.h"

class CameraClient : public ZmqClient
{
public:
	// Connect to camera before call other functions.
	bool connect(const std::string& ip) 
	{
		if (setAddr(ip, kImagePort, 10000))
		{
			if (getCameraIntri().isZero())
			{
				std::cout << "Failed to connect the camera! Please make sure the IP address is correct!";
				return false;
			}
		}
		return true; 
	}

	// Depth image type: CV_32FC1
	cv::Mat captureDepthImg();

	// Color image type: CV_8UC3
	cv::Mat captureColorImg();  

	// Units of point cloud: meter
	pcl::PointCloud<pcl::PointXYZ> capturePointCloud();
	pcl::PointCloud<pcl::PointXYZRGB> captureRgbPointCloud();

	CameraIntri getCameraIntri();

	std::string getCameraId();
	std::string getCameraVersion();
	std::string getParameter(const std::string& paraName, std::string& error);
	std::string setParameter(const std::string& paraName, double value);
	std::string setParameter(const std::string& propertyName, const std::vector<int>& value);
	Json::Value getCameraInfo();
	cv::Size getColorImgSize();
	cv::Size getDepthImgSize();

private:
	std::string sendRequest(const Json::Value& sendRequest, Json::Value& info, std::string& error);
    bool sendImageRequest(const std::string& command, int imageType, std::string& image, double& scale);
	std::string sendParamterRequest(const Json::Value& request, const std::string& paramter);
	Json::Value getImgSize();
	const uint16_t kImagePort = 5577;
	const int SIZE_OF_JSON = 4;
};


#endif // CAMERACLIENT_H
