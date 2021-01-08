#include <iostream>
#include <opencv2/imgcodecs.hpp>

class sample
{
public:
	int parameter();
	int ImgAndCloud();

private:
	cv::Mat to8U(const cv::Mat& src);
};
