#include "CameraClient.h"
#include <opencv2/imgcodecs.hpp>
#include "CameraCmd.h"
#include <regex>
#include "json/json.h"

namespace
{
	constexpr int kSizeOfRoi = 4;
	constexpr int kSizeOfINT32 = 4;
	double readDouble(const std::string& data, const int pos)
	{
		if (pos + sizeof(double) > data.size()) return 0;
		std::string strFromQDataStream(data.data() + pos, sizeof(double));
		std::string str;
		str.resize(sizeof(double));
		for (int i = 0; i < sizeof(double); i++)
		{
			str[i] = strFromQDataStream[sizeof(double) - 1 - i];
		}
		double v;
		memcpy(&v, str.c_str(), sizeof(double));
		return v;
	}

	int readInt(const std::string& data, const int pos)
	{
		if (pos + kSizeOfINT32 > data.size()) return 0;
		std::string strFromQDataStream(data.data() + pos, kSizeOfINT32);
		std::string str;
		str.resize(kSizeOfINT32);
		for (int i = 0; i < kSizeOfINT32; i++)
		{
			str[i] = strFromQDataStream[kSizeOfINT32 - 1 - i];
		}
		int v;
		memcpy(&v, str.c_str(), kSizeOfINT32);
		return v;
	}

	cv::Mat asMat(const std::string& data, size_t offset = 0)
	{
		return cv::Mat(data.size() - offset, 1, cv::DataType<uchar>::type,
			const_cast<char*>((data.data() + offset)));
	}

	cv::Mat reorganizeImage(const cv::Mat& matC1)
	{
		if (matC1.empty())
			return cv::Mat();
		if (matC1.channels() != 1 || (matC1.rows % 3) != 0)
			return cv::Mat();

		std::vector<cv::Mat> channels;
		for (int i = 0; i < 3; i++) {
			channels.push_back(matC1(cv::Rect(0, i * matC1.rows / 3, matC1.cols, matC1.rows / 3)));
		}
		cv::Mat matC3;
		cv::merge(channels, matC3);
		return matC3;
	}

	cv::Mat rescaleImageC1(const std::string& data, double scale)
	{
		if (data.empty()) return {};
		cv::Mat bias16U =
			cv::imdecode(asMat(data), cv::ImreadModes::IMREAD_ANYDEPTH);
		cv::Mat bias32F = cv::Mat::zeros(bias16U.size(), CV_32FC1);
		bias16U.convertTo(bias32F, CV_32FC1);
		cv::Mat mat32F =
			bias32F + cv::Mat(bias32F.size(), bias32F.type(), cv::Scalar::all(-Encode32FBias));
		return scale == 0 ? cv::Mat() : mat32F / scale;
	}
	
	cv::Mat rescaleImageC3(const std::string& data, double scale)
	{
		if (data.empty()) return cv::Mat();
		cv::Mat matC1 = cv::imdecode(asMat(data), cv::ImreadModes::IMREAD_ANYDEPTH);
		cv::Mat bias16UC3 = reorganizeImage(matC1);
		cv::Mat bias32F = cv::Mat::zeros(bias16UC3.size(), CV_32FC3);
		bias16UC3.convertTo(bias32F, CV_32FC3);
		cv::Mat depth32F =
			bias32F + cv::Mat(bias32F.size(), bias32F.type(), cv::Scalar::all(-Encode32FBias));
		cv::Mat rel = depth32F / scale;	
		return rel;
	}

	vector<string> splitStr(const string& str, const string& delim) {
		vector<string> res;
		if ("" == str) return res;
		char* strs = new char[str.length() + 1];
		strcpy(strs, str.c_str());

		char* d = new char[delim.length() + 1];
		strcpy(d, delim.c_str());

		char* p = strtok(strs, d);
		while (p) {
			string s = p;
			res.push_back(s);
			p = strtok(NULL, d);
		}

		return res;
	}

	vector<int> find_number_pos(const string &str) {
		vector<int> rel;
		if (str.size() == 0) {
			rel.push_back(-1);
			rel.push_back(-1);
		}
		else {
			for (int i = 0; i < str.size(); i++)
			{
				if (str[i] >= '0' && str[i] <= '9') {
					rel.push_back(i);
					break;
				}
			}
			for (int i = str.size() - 1; i >= 0; i--)
			{
				if (str[i] >= '0' && str[i] <= '9') {
					rel.push_back(i);
					break;
				}
			}
		}
		return rel;
	}
	
	bool isCommandValid(const std::string& response)
	{
		return response.find("Unsupported command.") == std::string::npos;
	}

	bool isResponseError(const std::string& response, Json::Value& info, std::string& error)
	{
		Json::Reader reader;
		reader.parse(response.substr(SIZE_OF_JSON, response.size() - SIZE_OF_JSON), info);
		if (info.isMember("err_msg") && info["err_msg"] != Json::Value(""))
		{
			error = info["err_msg"].toStyledString();
			return true;
		}
		return false;
	}
}

std::string CameraClient::sendRequest(const Json::Value& request, Json::Value& info, std::string& error)
{
	Json::FastWriter fwriter;
	std::string response = sendReq(fwriter.write(request));
	if (response.empty())
	{
		error = "Empty response!";
		return "";
	}
	if (!isCommandValid(response))
	{
		error = "Unsupported command! Please make sure to send the correct command!";
		return "";
	}
	Json::Reader reader;
	if (isResponseError(response, info, error))
	{
		return "";
	}
	return response;
}

cv::Mat CameraClient::captureDepthImg()
{
	std::string depthImage;
	double scale;
	if(!sendImageRequest(Command::CaptureImage, ImageType::DEPTH, depthImage, scale) || depthImage.empty()) //size()
	{
		std::cout << "Client depth image is empty!" << std::endl;
		return cv::Mat();
	}
	std::cout << "Depth image captured!" << std::endl;
	return rescaleImageC1(depthImage, scale);
}

cv::Mat CameraClient::captureColorImg()
{
	std::string colorImage;
	double scale;
	if(!sendImageRequest(Command::CaptureImage, ImageType::COLOR, colorImage, scale) || colorImage.empty()) //size()
	{
		std::cout << "Client color image is empty!" << std::endl;
		return cv::Mat();
	}
	std::cout << "Color image captured!" << std::endl;
	return cv::imdecode(asMat(colorImage), cv::ImreadModes::IMREAD_COLOR);
}

pcl::PointCloud<pcl::PointXYZRGB> CameraClient::captureRgbPointCloud()
{
	std::string depthImage;
	double scale;
	if (!sendImageRequest(Command::CaptureImage, ImageType::MatXYZ, depthImage, scale) || depthImage.empty()) //size()
	{
		std::cout << "The point cloud is empty!" << std::endl;
		return {};
	}
	return PointCloudTools::getRgbCloudFromDepthC3(rescaleImageC3(depthImage, scale), captureColorImg());
}

pcl::PointCloud<pcl::PointXYZ> CameraClient::capturePointCloud()
{
	std::string depthImage;
	double scale;
	if (!sendImageRequest(Command::CaptureImage, ImageType::MatXYZ, depthImage, scale) || depthImage.empty())  //size()
	{
		std::cout << "The point cloud is empty!" << std::endl;
		return {};
	}
	return PointCloudTools::getCloudFromDepthC3(rescaleImageC3(depthImage, scale));
}

bool CameraClient::sendImageRequest(const std::string& command, int imageType, std::string& image, double& scale)
{
	Json::Value request;
	request[Service::cmd] = Json::Value(command);
	request[Service::image_type] = Json::Value(imageType);
	std::string error;
	Json::Value info;
	std::string response = sendRequest(request, info, error);
	if (response.empty())
	{
		std::cout << "error: " << error;
		return false;
	}
	int jsonSize = readInt(response, 0);
	scale = readDouble(response, jsonSize + SIZE_OF_JSON);
	int imageSize = readInt(response, jsonSize + SIZE_OF_JSON + SIZE_OF_SCALE);
	int imageBegin = jsonSize + SIZE_OF_JSON + SIZE_OF_SCALE + kSizeOfINT32;
	image = response.substr(imageBegin, imageSize);
	return true;
}

CameraIntri CameraClient::getCameraIntri()
{
	Json::Value request;
	request[Service::cmd] = Json::Value(Command::GetCameraIntri);
	std::string error;
	Json::Value info;
	std::string response = sendRequest(request, info, error);
	if (response.empty())
	{
		std::cout << "error: " << error;
		return CameraIntri();
	}

	Json::Value intriValue = info["camera_intri"]["intrinsic"];
	std::string originStr = intriValue.toStyledString();
	vector<int> pos = find_number_pos(originStr);
	vector<std::string> intriArray = splitStr(originStr.substr(pos[0],pos[1] - pos[0]), ",");
	
	CameraIntri intri;
	intri.fx = std::stod(intriArray[0].c_str());
	intri.fy = std::stod(intriArray[1].c_str());
	intri.u = std::stod(intriArray[2].c_str());
	intri.v = std::stod(intriArray[3].c_str());

	std::cout.precision(17);
	std::cout << "fx = " << intri.fx << std::endl
		<< "fy = " << intri.fy << std::endl
		<< "u = " << intri.u << std::endl
		<< "v = " << intri.v << std::endl;

	return intri;
}

Json::Value CameraClient::getCameraInfo()
{
	Json::Value request;
	request[Service::cmd] = Json::Value(Command::GetCameraInfo);
	std::string error;
	Json::Value info;
	std::string response = sendRequest(request, info, error);
	if (response.empty()) 
	{
		std::cout << "error: " << error;
		return {};
	}
	return info["camera_info"];
}

std::string CameraClient::getCameraId()
{
	Json::Value info = getCameraInfo();
	return  info.empty() ? "" : info["eyeId"].toStyledString();
}

std::string CameraClient::getCameraVersion()
{
	Json::Value info = getCameraInfo();
	return  info.empty() ? "" : info["version"].toStyledString();
}

Json::Value CameraClient::getImgSize()
{
	Json::Value request;
	request[Service::cmd] = Json::Value(Command::GetImageFormat);
	std::string error;
	Json::Value info;
	std::string response = sendRequest(request, info, error);
	if (response.empty())
	{
		std::cout << "error: " << error;
		return {};
	}
	return info[Service::image_format];
}

cv::Size CameraClient::getColorImgSize()
{
	Json::Value imgSize = getImgSize();
	if (imgSize.empty())
		return { 0,0 };
	Json::Value size2d = imgSize[Service::size2d];
	return { size2d.get(Json::Value::ArrayIndex(0), 0).asInt(), size2d.get(Json::Value::ArrayIndex(1), 0).asInt() };
}

cv::Size CameraClient::getDepthImgSize()
{
	Json::Value imgSize = getImgSize();
	if (imgSize.empty())
		return { 0,0 };
	Json::Value size3d = imgSize[Service::size3d];
	return { size3d.get(Json::Value::ArrayIndex(0), 0).asInt(), size3d.get(Json::Value::ArrayIndex(1), 0).asInt() };
}

std::string CameraClient::getParameter(const std::string& paraName, std::string& error)
{
	Json::Value request;
	Json::FastWriter fwriter;
	request[Service::cmd] = Command::GetCameraParams;
	request[Service::property_name] = paraName;
	Json::Value info;
	std::string response = sendRequest(request, info, error);
	if (response.empty())
	{
		std::cout << "error: " << error;
		return "";
	}
	const int configId = info["camera_config"]["current_idx"].asInt();
	Json::Value configs = info["camera_config"]["configs"][configId];

	if (configs.isMember(paraName)) {
		return configs[paraName].toStyledString();
	}
	else {
		error = "Property not exist";
		return "";
	}
}

std::string  CameraClient::setParameter(const std::string& paramter, double value)
{
	Json::Value request;
	request[Service::cmd] = Command::SetCameraParams;
	request[Service::camera_config][paramter] = value;
	request[Service::persistent] = true;
	return sendParamterRequest(request, paramter);
}

std::string  CameraClient::setParameter(const std::string& paramter, const std::vector<int>& values)
{
	Json::Value request;
	request[Service::cmd] = Command::SetCameraParams;
	request[Service::persistent] = true;
	Json::Value valueToSet;
	if ("roi" == paramter && kSizeOfRoi == values.size()) {
		valueToSet["Height"] = values.at(0);
		valueToSet["Width"] = values.at(1);
		valueToSet["X"] = values.at(2);
		valueToSet["Y"] = values.at(3);
		request[Service::camera_config][paramter] = valueToSet;
	}
	else
	{
		return "Failed to set ROI! Please enter the correct ROI!";
	}
	return sendParamterRequest(request, paramter);
}

std::string CameraClient::sendParamterRequest(const Json::Value& request, const std::string& paramter)
{
	std::string error;
	Json::Value info;
	std::string response = sendRequest(request, info, error);
	if (response.empty())
	{
		std::cout << "error: " << error;
		return "Failed to set " + paramter + "!";
	}
	else
	{
		return "Set " + paramter + " successfully!";
	}
}
