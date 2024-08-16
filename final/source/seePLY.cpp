#include "../inc/PCLHead.h"
#include <pcl/surface/mls.h>
#include <iostream>

using namespace std;
using namespace pcl;
typedef PointXYZ PointT;
int main(void)
{
    PointCloud<PointXYZ>::Ptr sphere(new PointCloud<PointXYZ>), source(new PointCloud<PointXYZ>);

    visualization::PCLVisualizer viewer("viewer");

    int v1(0); // 创建视口v1
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    io::loadPLYFile("/home/dxy/Downloads/2024.2.28kinect_pthread_success/testcloud/SPHERE0.ply", *sphere);
    viewer.addPointCloud(sphere, "sphere");
    viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0.5, 0, 0.5, "sphere", v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sphere", v1);
    viewer.addCoordinateSystem(50);
    // io::loadPLYFile("/home/ddxy/Downloads/视觉部分/kinect/camera/testcloud/source_downsampled.ply", *source);
    MovingLeastSquares<PointXYZ, PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setPolynomialOrder(true);
    // io::savePLYFile("/home/ddxy/Downloads/视觉部分/kinect/camera/testcloud/sphere6*.ply", *sphere);

    PointT center0;
    center0.x = 0;
    center0.y = 0;
    center0.z = 0;
    PointT center1;
    center1.x = 0;
    center1.y = 0;
    center1.z = 0;
    PointT center2;
    center2.x = 0;
    center2.y = 0;
    center2.z = 0;
    PointT center3;
    center3.x = 0;
    center3.y = 0;
    center3.z = 0;
    PointT center4;
    center4.x = 0;
    center4.y = 0;
    center4.z = 0;
    PointT center5;
    center5.x = 0;
    center5.y = 0;
    center5.z = 0;
    PointT center6;
    center6.x = 0;
    center6.y = 0;
    center6.z = 0;
    PointT center7;
    center7.x = 0;
    center7.y = 0;
    center7.z = 0;
    PointT center8;
    center8.x = 0;
    center8.y = 0;
    center8.z = 0;
    PointT center9;
    center9.x = 0;
    center9.y = 0;
    center9.z = 0;
    PointT center10;
    center10.x = 0;
    center10.y = 0;
    center10.z = 0;
    for (auto c : sphere->points)
    {
        center0.x += c.x;
        center0.y += c.y;
        center0.z += c.z;
    }
    center0.x /= sphere->points.size();
    center0.y /= sphere->points.size();
    center0.z /= sphere->points.size();

    for (auto c : sphere->points)
    {
        center1.x += c.x * c.x;
        center1.y += c.y * c.y;
        center1.z += c.z * c.z;
    }
    center1.x /= sphere->points.size();
    center1.y /= sphere->points.size();
    center1.z /= sphere->points.size();

    for (auto c : sphere->points)
    {
        center2.x += c.x * c.y;
        center2.y += c.x * c.z;
        center2.z += c.y * c.z;
    }
    center2.x /= sphere->points.size();
    center2.y /= sphere->points.size();
    center2.z /= sphere->points.size();
    for (auto c : sphere->points)
    {
        center3.x += c.x * c.x * c.x;
        center3.y += c.x * c.x * c.y;
        center3.z += c.x * c.x * c.z;
    }
    center3.x /= sphere->points.size();
    center3.y /= sphere->points.size();
    center3.z /= sphere->points.size();
    for (auto c : sphere->points)
    {
        center4.x += c.x * c.y * c.y;
        center4.y += c.x * c.z * c.z;
        center4.z += c.y * c.y * c.y;
    }
    center4.x /= sphere->points.size();
    center4.y /= sphere->points.size();
    center4.z /= sphere->points.size();
    for (auto c : sphere->points)
    {
        center5.x += c.y * c.y * c.z;
        center5.y += c.y * c.z * c.z;
        center5.z += c.z * c.z * c.z;
    }
    center5.x /= sphere->points.size();
    center5.y /= sphere->points.size();
    center5.z /= sphere->points.size();
    center6.x = center1.x - center0.x * center0.x;
    center6.y = center2.x - center0.x * center0.y;
    center6.z = center2.y - center0.x * center0.z;
    center7.x = center2.x - center0.x * center0.y;
    center7.y = center1.y - center0.y * center0.y;
    center7.z = center2.z - center0.y * center0.z;
    center8.x = center2.y - center0.x * center0.z;
    center8.y = center2.z - center0.y * center0.z;
    center8.z = center1.z - center0.z * center0.z;
    center9.x = center3.x - center0.x * center1.x + center4.x - center0.x * center1.y + center4.y - center0.x * center1.z;
    center9.y = center3.y - center0.y * center1.x + center4.z - center0.y * center1.y + center5.y - center0.y * center1.z;
    center9.z = center3.z - center0.z * center1.x + center5.x - center0.z * center1.y + center5.z - center0.z * center1.z;
    center10.x = center9.x / 2;
    center10.y = center9.y / 2;
    center10.z = center9.z / 2;

    Eigen::Matrix3f B;
    B << center6.x, center6.y, center6.z,
        center7.x, center7.y, center7.z,
        center8.x, center8.y, center8.z;
    Eigen::Vector3f b(center10.x, center10.y, center10.z);
    Eigen::Vector3f C = B.colPivHouseholderQr().solve(b);
    std::cout << C << std::endl;
    PointT center;
    center.x = C.x();
    center.y = C.y();
    center.z = C.z();
    viewer.addSphere(center, 5, 0, 0, 1, "center", 0);
    viewer.addCoordinateSystem(50);
    while (!viewer.wasStopped())
    {
        viewer.spin();
    }

    return 0;
}
