#include "CameraClient.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/persistence.hpp>
#include "sample.h"

inline bool isApprox0(double d, double epsilon = DBL_EPSILON) {return std::fabs(d) <= epsilon;}

cv::Mat sample::to8U(const cv::Mat& src) 
{
	if (src.empty()) return {};
	double minV, maxV;
	cv::minMaxLoc(src, &minV, &maxV);
	cv::Mat dst;
	src.convertTo(dst, CV_8U, isApprox0(maxV) ? 1 : 255.0 / maxV);
	return dst;
}

//In this sample, we will show how to use a camera to take a 2d color image and save it on the disk
int sample::ImgAndCloud()
{
	//Before we do anything, we always need to connect to camera by ip.
	CameraClient camera;
	std::string error;
	// Camera ip should be modified to actual ip address.
	const std::string cameraIp = "192.168.3.168";
	if (!camera.connect(cameraIp)) return -1; //return -1 if connection to camera fails
	std::cout 
		<< "Camera ID: " << camera.getCameraId() << std::endl
		<< "Version: " << camera.getCameraVersion() << std::endl 
		<< "Color Image Size: " << camera.getColorImgSize() << std::endl
		<< "Depth Image Size: " << camera.getDepthImgSize() << std::endl; //get and print some information about camera device

	cv::Mat color = camera.captureColorImg(); //capture a 2d image and it will be stored as cv matrix
	if (color.empty()) std::cout << "empty error" << std::endl;
	else
	{
		cv::imwrite("./color.jpg", color); //save the color image to the disk
	}

	cv::Mat depth = camera.captureDepthImg(); //capture a depth image and it will be stored as cv matrix
	// cv::imwrite does not support 32F, so store the depth image through cv::FileStorage
	std::string savePath("./images.yml"); //set the target path you want to save the depth image, end with .yml
	cv::FileStorage fs(savePath, cv::FileStorage::WRITE); //initialize FileStorage
	if (depth.empty()) std::cout << "empty error" << std::endl;
	else
	{
		write(fs, "depth", depth); //save the depth image to the disk
	}

	std::cout << "Generating point cloud image" << std::endl;
	const pcl::PointCloud<pcl::PointXYZRGB> rgbCloud = camera.captureRgbPointCloud();
	PointCloudTools::saveRgbPointCloud("./rgbCloud.ply", rgbCloud); //can be .ply or .pcd, both works.

    // Read the images from saved files and show them
	std::cout << "Rendering images......." << std::endl;
	cv::Mat savedColorImage = cv::imread("./color.jpg"); //read out the saved color image

	if (savedColorImage.empty()) std::cout << "Save failed" << std::endl;
	else {
		cv::namedWindow("saved color image", cv::WINDOW_NORMAL); //create a window for showing images
		cv::imshow("saved color image", savedColorImage);
		cv::waitKey(10000); //show the color image and wait for 10s or key inputs
	}
	
	cv::Mat savedDepthImage;
	fs.open(savePath, cv::FileStorage::READ); //reopen the saved .yml file and get ready for reading out the saved depth image
	fs["depth"] >> savedDepthImage; //read out the depth image
	if (savedDepthImage.empty()) std::cout << "Save failed" << std::endl;
	else {
		cv::namedWindow("saved depth image", cv::WINDOW_NORMAL); //create a window for showing images
		cv::imshow("saved depth image", to8U(savedDepthImage)); //cv::imshow does not support 32F format, so convert it to 8U first 
		cv::waitKey(10000); //show the depth image and wait for 10s or key inputs
	}

	cv::destroyAllWindows();

	return 0;
}


