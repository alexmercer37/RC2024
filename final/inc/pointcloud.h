/*
 * @Author: Tommy0929
 * @Date: 2024-03-06 11:12:38
 * @LastEditors: Tommy0929 tommy07210728@163.com
 * @FilePath: /final/inc/pointcloud.h
 * WHUROBOCON_SAVED!!!
 */
#ifndef _POINTCLOUD_H
#define _POINTCLOUD_H
#include <iostream>
#include <k4a/k4a.hpp>
#include "OpencvHead.h"
#include "PCLHead.h"
#include "uart.h"
#include <time.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/console/parse.h>

typedef struct Pointcloud
{
    float x;
    float y;
    float z;

} Pointcloud;
class lcloud
{
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr source;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_downsampled;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sphere;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudT;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_sample;
    pcl::ConditionalRemoval<pcl::PointXYZ> filt;
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    //pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normal;
    pcl::PointIndices::Ptr inliers_sphere;
    pcl::ModelCoefficients::Ptr coefficients_sphere;
    pcl::PassThrough<pcl::PointXYZ> pass;

    int count;

public:
    lcloud();

    void getMaskAccordingToColor(const cv::Mat &cv_rgbimg, cv::Mat &mask);

    void getXYZPointCloud(const k4a::transformation &k4aTransformation, const k4a::calibration &k4aCalibration, const cv::Mat &cv_depth);

    void getPLY(__u8 buff);

    void getPLY(Pointcloud &pointcloud);

    void clearCloud();
};
#endif
