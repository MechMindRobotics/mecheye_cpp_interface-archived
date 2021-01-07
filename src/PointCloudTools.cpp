#include "PointCloudTools.h"
#include <pcl/io/ply_io.h>
#include <opencv2/imgcodecs.hpp>

pcl::PointCloud<pcl::PointXYZ> PointCloudTools::getCloudFromDepth(cv::Mat& depth,
                                                                  const CameraIntri& intri)
{
    if (depth.empty())
    {
        std::cout << "Depth image is empty." << std::endl;
        return {};
    }
    if (depth.depth() != CV_32F)
    {
        std::cout << "Depth image data type is wrong : " << depth.depth() << std::endl;
        return {};
    }
    pcl::PointCloud<pcl::PointXYZ> cloud(depth.cols, depth.rows);
    depth.forEach<float>([&](const float& val, const int* pos) {
        const int r = pos[0];
        const int c = pos[1];
        const pcl::PointXYZ p = mapPoint2dToPoint3d(r, c, val, intri);
        if (pcl::isFinite(p))
        {
            cloud.at(c, r) = p;
        }
    });
    return cloud;
}

void PointCloudTools::savePointCloud(const std::string& filePath,
                                     const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    pcl::PLYWriter().write(filePath, cloud);
}

pcl::PointXYZ PointCloudTools::mapPoint2dToPoint3d(int r, int c, float depth, const CameraIntri &intri)
{
    if (intri.isZero()) return {};
    pcl::PointXYZ p;
    const double x = (c - intri.u);
    const double y = (r - intri.v);
    p.x = 0.001 * depth * x / intri.fx; // mm to m
    p.y = 0.001 * depth * y / intri.fy;
    p.z = 0.001 * depth;
    return p;
}
